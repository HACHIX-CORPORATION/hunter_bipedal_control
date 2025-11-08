#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
from stl import mesh
import argparse

def mirror_stl_xz(input_filename, output_filename=None):
    """
    STLファイルをXZ面に関して反転（Y軸方向に反転）する
    
    Parameters:
    -----------
    input_filename : str
        入力STLファイルのパス
    output_filename : str, optional
        出力STLファイルのパス。指定がない場合は入力ファイル名の先頭に "mirrored_" を付加
    
    Returns:
    --------
    str : 保存したファイルのパス
    """
    # 入力ファイルの確認
    if not os.path.isfile(input_filename):
        print(f"エラー: 指定されたファイルが存在しません: {input_filename}")
        sys.exit(1)
    
    # 出力ファイル名の設定
    if output_filename is None:
        dirname = os.path.dirname(input_filename)
        basename = os.path.basename(input_filename)
        output_filename = os.path.join(dirname, f"mirrored_{basename}")
    
    # STLファイルを読み込み
    model = mesh.Mesh.from_file(input_filename)
    
    # XZ面に関して反転（Y座標の符号を反転）
    # 全ての頂点のY座標に-1を掛ける
    model.vectors[:, :, 1] *= -1
    
    # 法線ベクトルも反転する必要があるため、面の頂点の順序を反転
    # これによって法線の向きが逆になる
    model.vectors = model.vectors[:, ::-1, :]
    
    # 変換後のSTLを保存
    model.save(output_filename)
    
    print(f"XZ面ミラーリング完了: {output_filename}")
    return output_filename

def main():
    # コマンドライン引数の解析
    parser = argparse.ArgumentParser(description='STLファイルをXZ面に関してミラーリングする')
    parser.add_argument('input_file', help='入力STLファイルのパス')
    parser.add_argument('-o', '--output', help='出力STLファイルのパス（省略可能）')
    args = parser.parse_args()
    
    # ミラーリング実行
    mirror_stl_xz(args.input_file, args.output)

if __name__ == "__main__":
    main() 