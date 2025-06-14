## 環境構築
### gitが入っているかの確認
ターミナルに次のように入力します。
``` shell
git -v
```
Enterを押して
``` shell
git version 2.45.1.windows.1
```
と表示されれば大丈夫です。

### リポジトリのクローン
ターミナルでクローン死体ディレクトリで次のコマンドを打ちます。
``` 　shell
git clone https://github.com/Tachyon-RCJ/Robot_code_world.git
```
リポジトリがローカルにクローンされるはずです。

### VScodeでArduinoを使用するためのライブラリ
拡張機能のタブで「Arduino Community Edition」と検索し、インストールします。

また、今回はC++も使用するので「C/C++」というMicrosoftから出ている拡張機能もインストールします。

以上で、VScode上でArduinoが書ける準備が整いました。