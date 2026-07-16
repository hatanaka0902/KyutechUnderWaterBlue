"""
Control/imu_stream.py の単体テスト。

対象クラス: ImuStream(listen_ip, listen_port, stale_timeout)
    - start()/stop()          : TCPサーバをバックグラウンドスレッドで起動/停止
    - _parse_and_store(line)  : ESP32からのCSV行をパースして最新値として保持
    - get_latest()             : 直近値を返す(stale_timeout超過ならNone)

ここでの「実機」はBlueROV本体ではなくIMU用ESP32だが、実際のESP32を使わず
localhost上のTCPクライアントで疑似的な接続・データ送信を行うことで、
実機ESP32なしにプロトコル(MEASUREコマンド送信、CSV13フィールド、
qw,qx,qy,qz並べ替え、stale_timeout)を検証する。BlueROV自体は不要。

実行方法:
    python test/test_imu_stream.py
"""

import os
import socket
import sys
import time
import unittest

import numpy as np

_CONTROL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "Control")
if _CONTROL_DIR not in sys.path:
    sys.path.insert(0, _CONTROL_DIR)

from imu_stream import ImuStream  # noqa: E402


def _free_port():
    """OSに空きTCPポートを1つ割り当てさせて番号を取得する(テスト間の競合を避ける)。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class ImuStreamTestBase(unittest.TestCase):
    """各テストで新しいポート・新しいImuStreamインスタンス・新しいクライアント接続を
    用意する共通セットアップ。ESP32実機の代わりに素のTCPクライアントを使う。"""

    def setUp(self):
        self.port = _free_port()
        self.stream = ImuStream(listen_ip="127.0.0.1", listen_port=self.port, stale_timeout=0.3)
        self.stream.start()
        # _accept_loop がバインド・listenを完了するまでの猶予
        time.sleep(0.1)
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.settimeout(2.0)
        self.client.connect(("127.0.0.1", self.port))

    def tearDown(self):
        try:
            self.client.close()
        finally:
            self.stream.stop()

    def _send_line(self, line: str):
        self.client.sendall((line + "\n").encode("utf-8"))

    def _wait_for_latest(self, timeout=1.0):
        """get_latest()がNoneでなくなるまで短時間ポーリングする(受信スレッドの非同期性対策)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            latest = self.stream.get_latest()
            if latest is not None:
                return latest
            time.sleep(0.02)
        return None


class TestConnectionHandshake(unittest.TestCase):
    """接続確立直後の挙動(IMU_CALIB_AND_REC.ino準拠プロトコル)。"""

    def setUp(self):
        self.port = _free_port()
        self.stream = ImuStream(listen_ip="127.0.0.1", listen_port=self.port, stale_timeout=0.3)
        self.stream.start()
        time.sleep(0.1)

    def tearDown(self):
        self.stream.stop()

    def test_server_sends_measure_command_immediately_on_connect(self):
        """目的: ESP32が接続した直後に、確認往復(GET_ONCE等)を経由せず即座に
        "MEASURE\\n" が送信されることを確認する。
        役割: imu_stream.pyのdocstringに明記された「即座にMEASUREを送る」設計
        (リアルタイム制御のための往復省略)が実際に実装されていることの確認。
        結果が示すこと: クライアントが接続後に最初に受信するバイト列が b"MEASURE\\n"。"""
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(2.0)
        try:
            client.connect(("127.0.0.1", self.port))
            received = client.recv(4096)
            self.assertEqual(received, b"MEASURE\n")
        finally:
            client.close()


class TestParseAndStore(ImuStreamTestBase):
    """CSV行のパース・保持ロジック(_parse_and_store経由、get_latest()で確認)。"""

    def test_valid_13_field_line_is_parsed_correctly(self):
        """目的: 正しい13フィールドのCSV行が accel/gyro/mag/quat に正しく分解されることを確認する。
        役割: function.get_acceleration_data()/get_gyro_data()/get_orientation_measurement()が
        直接依存する、センサ融合の入口となるパース処理の正しさの確認。
        結果が示すこと: "Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz,Qx,Qy,Qz,Qw" の順で送った値が、
        get_latest()の'accel'/'gyro'に正しい順序で入り、'quat'は[Qw,Qx,Qy,Qz]の
        順(w成分を先頭)に並べ替えられている。"""
        self._send_line("1.0,2.0,3.0,0.1,0.2,0.3,4.0,5.0,6.0,0.10,0.20,0.30,0.90")
        latest = self._wait_for_latest()
        self.assertIsNotNone(latest, "get_latest()がタイムアウトまでにNoneのまま(受信できていない)")
        np.testing.assert_allclose(latest["accel"], [1.0, 2.0, 3.0])
        np.testing.assert_allclose(latest["gyro"], [0.1, 0.2, 0.3])
        np.testing.assert_allclose(latest["mag"], [4.0, 5.0, 6.0])
        # 送信順は Qx,Qy,Qz,Qw だが、保持時は [qw,qx,qy,qz] に並べ替えられる
        np.testing.assert_allclose(latest["quat"], [0.90, 0.10, 0.20, 0.30])

    def test_non_csv_status_lines_are_ignored_without_crashing(self):
        """目的: "HELLO_FROM_ESP" 等の非データ行(カンマ13個の数値行にならない行)が
        エラーなく無視されることを確認する。
        役割: imu_stream.pyのコメントで説明されている「非データ行は自然に弾かれる」
        設計が実際に機能し、受信スレッドが例外で落ちないことの確認。
        結果が示すこと: ステータス行を送った直後はget_latest()がNoneのままで、
        続けて有効な行を送れば正常にパースされる(スレッドが継続動作している)。"""
        self._send_line("HELLO_FROM_ESP")
        self._send_line(">> MEASURE")
        self.assertIsNone(self.stream.get_latest())

        self._send_line("1.0,2.0,3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0")
        latest = self._wait_for_latest()
        self.assertIsNotNone(latest)
        np.testing.assert_allclose(latest["accel"], [1.0, 2.0, 3.0])

    def test_line_with_wrong_field_count_is_ignored(self):
        """目的: カンマの数が13個ちょうどでない行(送信中の欠損等)が無視されることを確認する。
        役割: フィールド数チェック(len(parts)!=13)が正しく機能し、不完全なデータで
        誤ったセンサ値が保持されないことの確認。
        結果が示すこと: 12フィールドの行を送ってもget_latest()はNoneのまま、
        続く正常な13フィールド行だけが反映される。"""
        self._send_line("1.0,2.0,3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0")  # 12個
        self.assertIsNone(self.stream.get_latest())

        self._send_line("9.0,9.0,9.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0")
        latest = self._wait_for_latest()
        self.assertIsNotNone(latest)
        np.testing.assert_allclose(latest["accel"], [9.0, 9.0, 9.0])

    def test_non_numeric_field_is_ignored(self):
        """目的: 数値に変換できないフィールドを含む行(ノイズ混入等)が無視されることを確認する。
        役割: float(p) の ValueError を try/except で握り潰して「無視して継続」する
        フェイルセーフが機能していることの確認。ここが無いと1行のノイズで
        受信スレッド全体がクラッシュし、IMUデータが完全に途絶する。
        結果が示すこと: 数値でないフィールドを含む行を送ってもget_latest()はNoneのまま、
        以降のスレッド動作は継続する(次の正常行は正しく反映される)。"""
        self._send_line("a,b,c,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0")
        self.assertIsNone(self.stream.get_latest())

        self._send_line("2.0,2.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0")
        latest = self._wait_for_latest()
        self.assertIsNotNone(latest)


class TestStaleTimeout(ImuStreamTestBase):
    """stale_timeout: 直近受信からこの秒数を超えたら「データ無し」扱いにする鮮度管理。
    function.get_acceleration_data()等はNoneを見てRuntimeErrorを出すため、
    ここが機能しないと配線が抜けた古いデータをいつまでも「有効」として使い続けてしまう。"""

    def test_get_latest_returns_none_before_any_data(self):
        """目的: 一度もデータを受信していない状態でget_latest()がNoneを返すことを確認する。
        役割: 未接続・未受信の初期状態を安全に扱えていることの確認(いきなり不正な
        辞書やゼロ埋めデータを返さない)。
        結果が示すこと: 接続直後、何も送っていない時点でget_latest()はNone。"""
        self.assertIsNone(self.stream.get_latest())

    def test_get_latest_becomes_none_after_stale_timeout_elapses(self):
        """目的: 有効なデータ受信後、stale_timeout(このテストでは0.3秒)を超えて
        新規受信が無いとget_latest()がNoneに戻ることを確認する。
        役割: ESP32との接続が切れた/送信が止まった場合に、古いIMU値を使い続けて
        state_estimation()が誤った(古い)加速度・角速度で積分してしまう事故を防ぐ
        フェイルセーフの検証。
        結果が示すこと: 受信直後はNoneでない値が返るが、stale_timeout超過後はNone。"""
        self._send_line("1.0,1.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0")
        latest = self._wait_for_latest()
        self.assertIsNotNone(latest)

        time.sleep(0.45)  # stale_timeout=0.3を十分に超えて待つ
        self.assertIsNone(self.stream.get_latest())


if __name__ == "__main__":
    unittest.main(verbosity=2)
