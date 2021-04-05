# LOVOTの状態をチェックする
Raspberry piにUSBカメラを接続し、ネスト付近の動画を常に撮影できるようにします。LOVOTが一定時間充電されていない場合には研究室全体にその旨をメールします。

## 使い方
Raspberry pi上で、それぞれ別のターミナルで
```
$ roscore
$ rosrun usb_cam usb_cam_node
$ python ./check_lovot_status.py (_debug_view:=True)
```
_debug_view:=Trueにするとテンプレートマッチングの結果を可視化することができます。<br>
また、PCで実行するなど、USBカメラ以外のカメラが存在する場合は
```
$ rosrun usb_cam usb_cam_node _video_device:=/dev/video4
```
のようにUSBカメラを引数に設定します。USBカメラのパスは以下のようにして調べることができます。
```
$ v4l2-ctl --list-devices
UCAM-DLE300T: UCAM-DLE300T (usb-0000:06:00.3-3):
	/dev/video4
	/dev/video5

Integrated Camera: Integrated C (usb-0000:06:00.4-2.1):
	/dev/video0
	/dev/video1
	/dev/video2
	/dev/video3
```
上の例では`_video_device:=/dev/video4`または`_video_device:=/dev/video5`とすることでUSBカメラを指定することができます。


## Raspbrry pi3にUbuntu18をいれてセットアップするまで

### SDカードの準備（環境：Windows）

#### アプリの準備
以下のアプリをWindowsに入れておく
* 7-zip
    * .xzファイルを解凍するアプリ。[このリンク](https://sevenzip.osdn.jp/)からダウンロードする。だいたいの人は64ビット x64に該当するはず。
* Win32DiskImager
    * SDカードに書き込みを行うアプリ。[このリンク](https://win32-disk-imager.jp.uptodown.com/windows/download)からダウンロードする。

#### 書き込み手順
1. [このページ](http://www.cs.tohoku-gakuin.ac.jp/pub/Linux/RaspBerryPi/)からubuntu-mate-18.04.2-beta1-desktop-arm64+raspi3-ext4.img.xzを見つけてダウンロードする。
2. ダウンロードした.xzファイルを右クリックし、7-zip -> 展開で解凍する。.imgファイルが生成される。
3. SDカードをPCに接続する。Win32DiskImagerを開き、.imgファイルとデバイスを選択してWriteボタンを押すことでSDカードに書き込みが行われる。
4. 書き込みされたSDカードをRaspberry piに挿入し、以下の図のように接続する。ディスプレイにうまく表示されない場合はRaspberry piの電源を抜き差しするとうまくいくかもしれない。
5. 設定は基本的に全部「日本語」でOK。

### ROSをいれる
Raspberry pi上のターミナルで[Ubuntu install of ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)のページに従っていれる。
※Wi-Fiが繋がっている必要がある。遅い場合は有線推奨。

### PCからSSHできるようにする
#### SSHしたい側（ノートPC）で
```
$ ssh-keygen -t rsa
```
ひたすらEnter
```
$ cd ~/.ssh
$ cat id_rsa.pub
```
ssh-rsaからhoge@fuga まで（★）をコピー

* SSHされたい側（Raspberry pi3）で
1. .sshディレクトリに移動
```
$ cd .ssh
```
.sshディレクトリが存在しなければ作る
```
$ mkdir .ssh
```
2. authorized_keysファイルに書き込み
ファイルが存在しなければ作る。
```
touch authorized_keys
```
先程コピーしたPCの鍵（★）をファイルに書き込む。vimを使うなら
```
i 
★をペースト
esc
:wq
```
書き込みが完了したら
```
cat authorized_keys
```
を行い、書き込みができているかを確認する。

* Raspberry piのポートの設定変更
```
sudo raspi-config
```
して、SSHの項目をenableに設定する。設定が変更できたらRaspberry piを再起動する。

* SSHしてみる

※SSHできない場合は[トラブルシューティング](#トラブルシューティング)の項目を参照
以下、SSHしたい側のPCの名前をtork@ubuntu、Raspberry pi側の名前をlovotmonitor@lovotmonitor-desktopとする。
1. PCとRaspberry piを同じネットワークに繋いでおく。有線が確実。それぞれの端末でifconfigすれば今繋がっているネットワークを確認できる。
2. PCからSSHする
```
tork@ubuntu:~/$ ping lovotmonitor-desktop.local
```
上記コマンドでまずpingが通るか確認する。これができなければ何かがおかしい。
```
tork@ubuntu:~/$ ssh lovotmonitor@lovotmonitor-desktop.local
```
これでSSHできるはず

### トラブルシューティング
SSHできない場合は以下のコマンドでsshを入れ直してみる。
```
$ sudo apt-get --purge remove openssh-server
$ sudo apt-get install openssh-server
```

### SSHしてRaspberry piを動かす
ariesの鍵をRaspberry piに登録すると同じネットワークになくてもsshできて便利。
ariesにssh▷raspberry piにssh。この時、```ssh lovotmonitor@[IPアドレス]```として```ifconfig```して調べたアドレスを直接打たないと多分うまくいかない。<br>

ラズパイで動いているプログラムを確認する。
```
lovotmonitor@lovotmonitor-desktop:~$ ps a
```
例えばターミナルでroscore, usb_cam, check_lovot_status.pyを実行している場合は以下のように出力される。
```
  PID TTY      STAT   TIME COMMAND
 1001 tty7     Ssl+ 1632:03 /usr/lib/xorg/Xorg -core :0 -seat seat0 -auth /var/r
 1004 tty1     Ss+    0:00 /sbin/agetty -o -p -- \u --noclear tty1 linux
 2171 pts/1    Ss     0:00 bash
 2238 pts/2    Ss     0:00 bash
 7737 pts/0    Ss     0:01 bash
 9436 pts/2    Sl+   29:14 /usr/bin/python /opt/ros/melodic/bin/roscore
 9442 pts/1    Sl+  1393:48 /opt/ros/melodic/lib/usb_cam/usb_cam_node
 9486 pts/0    Sl+  4390:06 python ./check_lovot_status.py
14380 pts/3    Ss     0:00 -bash
14441 pts/3    R+     0:00 ps a
```

./check_lovot_status.pyを止めたい場合、```lovotmonitor@lovotmonitor-desktop:~$ kill 9486```すれば良い。

逆にssh先でプロセスを実行し続けたい場合はtmuxを使うのが良い。<br>
入ってなかったらインストールする。
```
lovotmonitor@lovotmonitor-desktop:~$ sudo apt install tmux
```
```
lovotmonitor@lovotmonitor-desktop:~$ tmux
```
セッションを開始したらプログラム実行
```
$ python ./check_lovot_status.py
```
プログラムを実行した状態で[Control + b]でコマンドモードに変更した後\[d]キーでセッションから抜けることができ、sshから抜けてもプログラムを実行した状態にできる。