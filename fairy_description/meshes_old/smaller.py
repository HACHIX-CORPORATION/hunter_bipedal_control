import os
import sys
from stl import mesh

def scale_stl(input_filename, output_dir="./scaled/", scale_factor=0.001):
    # 入力ファイルの確認
    if not os.path.isfile(input_filename):
        print(f"エラー: 指定されたファイルが存在しません: {input_filename}")
        sys.exit(1)
    
    # 出力ディレクトリの作成（存在しない場合）
    os.makedirs(output_dir, exist_ok=True)
    
    # 入力ファイル名から出力ファイルパスを作成
    base_name = os.path.basename(input_filename)
    output_filename = os.path.join(output_dir, base_name)

    # STLファイルを読み込み
    model = mesh.Mesh.from_file(input_filename)

    # スケール変換 (mm -> m)
    model.vectors *= scale_factor

    # 変換後のSTLを保存
    model.save(output_filename)
    print(f"スケール変換完了: {output_filename}")

if __name__ == "__main__":
    # コマンドライン引数の処理
    if len(sys.argv) != 2:
        print("使用法: python smaller.py <入力STLファイル>")
        sys.exit(1)

    input_file = sys.argv[1]
    scale_stl(input_file)
