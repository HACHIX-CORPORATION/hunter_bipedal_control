#!/usr/bin/env python3
"""
STLファイルのメッシュをリダクションするスクリプト
MeshLabのAPIを使用して、指定された削減率でメッシュを簡略化します。
"""

import os
import sys
import argparse

try:
    import pymeshlab
except ImportError:
    print("エラー: pymeshlabがインストールされていません。")
    print("インストール方法: pip install pymeshlab")
    sys.exit(1)

def reduce_mesh(input_file, output_file, target_reduction=0.1):
    """
    STLファイルのメッシュをリダクションする
    
    Args:
        input_file (str): 入力STLファイルのパス
        output_file (str): 出力STLファイルのパス
        target_reduction (float): 目標削減率 (0.1 = 10%に削減)
    """
    try:
        # MeshSetオブジェクトを作成
        ms = pymeshlab.MeshSet()
        
        # STLファイルを読み込み
        print(f"読み込み中: {input_file}")
        ms.load_new_mesh(input_file)
        
        # 元の面数を取得
        original_faces = ms.current_mesh().face_number()
        target_faces = int(original_faces * target_reduction)
        print(f"元の面数: {original_faces}")
        print(f"目標面数: {target_faces} ({target_reduction*100:.0f}%)")
        
        # Quadric Edge Collapse Decimationを使用してメッシュを簡略化
        ms.apply_filter('meshing_decimation_quadric_edge_collapse', 
                       targetfacenum=target_faces,
                       preservenormal=True,
                       preservetopology=True,
                       optimalplacement=True,
                       planarquadric=True,
                       qualitythr=0.3)
        
        # 結果を保存
        print(f"保存中: {output_file}")
        ms.save_current_mesh(output_file)
        
        # 最終的な面数を取得
        final_faces = ms.current_mesh().face_number()
        reduction_percentage = (1 - final_faces / original_faces) * 100
        print(f"最終面数: {final_faces} (削減率: {reduction_percentage:.1f}%)")
        print("メッシュリダクション完了")
        
    except Exception as e:
        print(f"エラーが発生しました: {str(e)}")
        return False
    
    return True

def batch_reduce(input_dir, output_dir, target_reduction=0.1):
    """
    ディレクトリ内のすべてのSTLファイルをバッチ処理
    """
    # 出力ディレクトリを作成
    os.makedirs(output_dir, exist_ok=True)
    
    # STLファイルを検索
    stl_files = [f for f in os.listdir(input_dir) 
                 if f.lower().endswith('.stl')]
    
    if not stl_files:
        print(f"STLファイルが見つかりません: {input_dir}")
        return
    
    print(f"{len(stl_files)}個のSTLファイルを処理します")
    
    # 各ファイルを処理
    for stl_file in stl_files:
        input_path = os.path.join(input_dir, stl_file)
        output_path = os.path.join(output_dir, stl_file)
        
        print(f"\n処理中: {stl_file}")
        reduce_mesh(input_path, output_path, target_reduction)

def main():
    parser = argparse.ArgumentParser(
        description='STLファイルのメッシュをリダクションします'
    )
    parser.add_argument('input', help='入力STLファイルまたはディレクトリ')
    parser.add_argument('-o', '--output', help='出力ファイルまたはディレクトリ')
    parser.add_argument('-r', '--reduction', type=float, default=0.1,
                        help='目標削減率 (デフォルト: 0.1 = 10%%に削減)')
    parser.add_argument('-b', '--batch', action='store_true',
                        help='ディレクトリ内のすべてのSTLファイルを処理')
    
    args = parser.parse_args()
    
    if args.batch:
        # バッチ処理モード
        if not os.path.isdir(args.input):
            print(f"エラー: ディレクトリが存在しません: {args.input}")
            sys.exit(1)
        
        output_dir = args.output or os.path.join(args.input, 'reduced')
        batch_reduce(args.input, output_dir, args.reduction)
    else:
        # 単一ファイル処理モード
        if not os.path.isfile(args.input):
            print(f"エラー: ファイルが存在しません: {args.input}")
            sys.exit(1)
        
        if args.output:
            output_file = args.output
        else:
            base_name = os.path.basename(args.input)
            name, ext = os.path.splitext(base_name)
            output_file = f"{name}_reduced{ext}"
        
        reduce_mesh(args.input, output_file, args.reduction)

if __name__ == "__main__":
    main()