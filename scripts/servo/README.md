# Dynamixel関連スクリプト

## Servoのテスト

### Dynamixel Wizard2

1. DynamixelWizard2のsetup wizardを公式サイトから落とす
2. Downloadsにあるはずなので、実行権限をchmodで付与しインストールする
3. このREADME.mdがあるディレクトリで下記を実行

```bash
$ make
```
これでsymlinkが張られ、`servo`コマンドでGUIを起動することができる。  
Waylandでうまく起動しない場合があるので環境変数でX11を使うことを強制している

単純に実行テストしたいときは`make launch`


