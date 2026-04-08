import os

# まとめるテキストファイルの名前
OUTPUT_FILE = "all_codes_combined.txt"

# 対象とする拡張子（提示いただいたリストに含まれる形式）
TARGET_EXTENSIONS = (".cpp", ".hpp", ".yaml", ".xml", ".txt", ".py")


def main():
    # スクリプトが置かれているディレクトリ（/home/amsl/ros2_ws/src/chibi26_c/ になる想定）
    base_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(base_dir, OUTPUT_FILE)

    # 書き込みモードでファイルを開く
    with open(output_path, "w", encoding="utf-8") as outfile:
        # base_dir以下のすべてのディレクトリとファイルを取得
        for root, dirs, files in os.walk(base_dir):
            for file in files:
                # 出力先のテキストファイル自身と、このスクリプト自身は除外
                if file == OUTPUT_FILE or file == os.path.basename(__file__):
                    continue

                # 対象の拡張子かどうかを判定
                if file.endswith(TARGET_EXTENSIONS):
                    # 絶対パスを作成
                    file_path = os.path.join(root, file)

                    # 1. 見やすく区切り線とファイルパスを書き込む
                    outfile.write(f"{'='*80}\n")
                    outfile.write(f"{file_path}\n")
                    outfile.write(f"{'='*80}\n\n")

                    # 2. ファイルの中身を読み込んで書き込む
                    try:
                        with open(file_path, "r", encoding="utf-8") as infile:
                            outfile.write(infile.read())
                    except Exception as e:
                        outfile.write(f"ファイルの読み込みエラー: {e}")

                    # 次のファイルとの間に空白行を入れて見やすくする
                    outfile.write("\n\n\n")

    print(f"完了しました！\n{output_path} に全コードがまとめられました。")


if __name__ == "__main__":
    main()
