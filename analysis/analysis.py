import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def load_and_preprocess(filepath: str) -> pd.DataFrame:
    """データの読み込みと前処理（座標文字列の数値化）を実行"""
    df = pd.read_csv(filepath)
    
    # XYZ座標がスペース区切りの文字列として格納されているため、分割してfloat型へ変換
    def parse_xyz(col_name):
        if col_name in df.columns:
            # 欠損値を考慮しつつ文字列を分割
            xyz_df = df[col_name].str.split(expand=True).astype(float)
            if xyz_df.shape[1] == 3:
                df[[f'{col_name}_x', f'{col_name}_y', f'{col_name}_z']] = xyz_df

    for col in ['pos_xyz', 'target_xyz', 'discovered_xyz']:
        parse_xyz(col)
        
    # 同一シード内での各状態における滞在時間を算出
    df['time_in_state'] = df.groupby('seed')['elapsed_time'].diff().shift(-1)
    
    return df

def analyze_state_transitions(df: pd.DataFrame):
    """状態遷移の頻度と各状態の平均滞在時間を分析"""
    print("=== 状態遷移頻度 (Top 10) ===")
    transitions = df.groupby(['from_state', 'to_state']).size().reset_index(name='count')
    transitions = transitions.sort_values('count', ascending=False)
    print(transitions.head(10).to_string(index=False))
    
    print("\n=== 各状態の平均滞在時間 (秒) ===")
    state_durations = df.groupby('to_state')['time_in_state'].mean().sort_values(ascending=False)
    print(state_durations.dropna().head(10))

def analyze_episode_performance(df: pd.DataFrame):
    """シード（試行）ごとの所要時間と成功/失敗傾向の分析"""
    print("\n=== シード別総経過時間の統計 ===")
    episode_durations = df.groupby('seed')['elapsed_time'].max()
    print(episode_durations.describe())
    
    # 経過時間の分布を可視化
    plt.figure(figsize=(10, 6))
    sns.histplot(episode_durations, bins=30, kde=True)
    plt.title('Distribution of Episode Durations by Seed')
    plt.xlabel('Total Elapsed Time (s)')
    plt.ylabel('Frequency')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()

def plot_trajectory(df: pd.DataFrame, target_seed: int = 0):
    """指定したシードのXY平面上での軌跡とターゲット位置を可視化"""
    seed_df = df[df['seed'] == target_seed]
    
    if 'pos_xyz_x' not in seed_df.columns:
        print("位置情報データがパースされていません。")
        return

    plt.figure(figsize=(8, 8))
    
    # エージェントの移動軌跡
    plt.plot(seed_df['pos_xyz_x'], seed_df['pos_xyz_y'], 
             marker='.', linestyle='-', markersize=4, alpha=0.7, label='Agent Trajectory')
    
    # 発見されたターゲットの位置（存在する場合）
    if 'discovered_xyz_x' in seed_df.columns:
        plt.scatter(seed_df['discovered_xyz_x'], seed_df['discovered_xyz_y'], 
                    color='red', marker='x', s=100, label='Discovered Target', zorder=5)
        
    plt.title(f'Agent Trajectory (XY Plane) - Seed: {target_seed}')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 実行制御
    file_path = "2026-07-18-56.csv"
    
    # 1. 前処理済みのデータフレームを取得
    processed_df = load_and_preprocess(file_path)
    
    # 2. 定量分析の出力
    analyze_state_transitions(processed_df)
    analyze_episode_performance(processed_df)
    
    # 3. 特定のシード（例: 0）の軌跡プロット
    plot_trajectory(processed_df, target_seed=0)