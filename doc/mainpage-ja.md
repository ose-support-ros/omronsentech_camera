# ROS上のOMRON SENTECHカメラSDK

## 1. 概要
omronsentech_cameraパッケージはオムロン センテック株式会社が提供するパッケージです。
本パッケージ対応カメラはLinux版のSentechSDK対応カメラと同様です。
カメラの機能や*GenICam GenApi*ノードのアクセスはSentechSDKのカメラ制御機能(Transport Layer) モジュール経由で行います。
*GenICam GenApi*ノードとROSにあるノードとは全く別の物であることを注意下さい。 各GenICam GenApiノードは、汎用インタフェースを介してアクセス可能なカメラデバイスの機能またはレジスタに対応しています。GenICam GenApiの詳細は[GenICam GenApiドキュメント（英語）]を参照下さい。

## 2. インストール
本パッケージはUbuntu 16.04 64ビット上のROS-Kineticで動作確認済みです。
SentechSDKのカメラ制御機能(Transport Layer) モジュールを利用するため、本パッケージをインストールする際は事前にSentechSDKをインストールする必要があります。下記のURLでSentechSDK (USB3VisionとGigEVision)がダウンロードできます：
https://sentech.co.jp/products/USB/software.html.

本パッケージは下記のROSパッケージが必要となります：

1. camera_info_manager
2. image_transport
3. message_generation
4. sensor_msgs
5. std_msgs
6. pluginlib
7. nodelet
8. roscpp
9. roslaunch
10. roslint
11. rospy

インストール手順として、catkinのワークスペースにomronsentech_camera.gitをクローンするかomronsentech_cameraのフォルダをコピーして下さい。 通常のROSパッケージとしてパッケージをビルドする前にSentechSDKの環境変数の設定を確認して下さい:

``$ echo $STAPI_ROOT_PATH``

環境変数が未設定の場合は、下記のコマンドを実行して下さい（SentechSDKを/opt/sentechにインストールした場合）：

 ``$ source /opt/sentech/.stprofile``

omronsentech_cameraの内部構成は以下のとおりです。
<pre>
/
|-./CMakeLists.txt
|-./nodelet_plugins.xml
|-./package.xml
|-./rosdoc.yaml
|-./doc/
|      |-./html/
|      |-./mainpage-en.md
|      |-./mainpage-ja.md
|      |-./manifest.yaml
|
|-./examples/
|      |-./cpp/grabber.cpp
|      |-./python/grabber.py
|
|-./include/
|      |-./omronsentech_camera/*.h
|                             
|-./launch/
|      |-./*.launch
|      |-./*.yaml
|
|-./msg/
|      |-./*.msg
|
|-./src/
|      |-./omronsentech_camera/*.cpp
|                             
|-./srv/
       |-./*.srv
</pre>

docフォルダにあるドキュメントを生成する場合、omronsentech_cameraフォルダで下記のコマンドを実行下さい：

 ``$ rosdoc_lite -o doc ``

 注意：Doxygenとrosdoc_liteパッケージが必要となります。

## 3. stcamera_nodeノード
パッケージにあるstcamera_nodeはパラメータ**camera_to_connect**により自動的に最初見つけたカメラ、指定したカメラや全カメラと接続し、画像を取得します。

stcamera_nodeノードの実行は次のコマンドのいずれかを実行下さい：

* ``$ rosrun omronsentech_camera stcamera_node``

又は

* ``$ roslaunch omronsentech_camera stcamera_node.launch``

ノードをstcamera_node.launchファイルスクリプトで起動するとカメラ接続パラメータがstcamera_node.yamlで簡単に設定できます。 パラメータ設定の詳細は次の章を参照下さい。

### 3.1. ノードパラメータ
パラメータの値は大文字と小文字を区別します。設定できるパラメータは下記の通りです：

  * **camera_to_connect** : 接続可能なカメラの設定。このパラメーターは、stcamera_nodeが実行される前に設定する必要があります。 実行中にこのパラメーターを変更しても、その変更は反映されません。設定無しの場合、最初に見つけたカメラを接続します。"all"を設定する場合、見つけた全てのカメラを接続します。CAMERA_ID又はCAMERA_MODEL(SERIAL)を設定する場合、指定したカメラのみと接続します。例：
    * ``camera_to_connect:[]``: 最初に見つけたカメラを接続します。
    * ``camera_to_connect:["all"]``: 見つけた全てのカメラを接続します。キーワード「all」が提示された場合、見つけた全てのカメラを接続しようとします。
    * ``camera_to_connect:["00:11:1c:f6:yy:xx","STC-MCS510U3V(00XXYY0)"]``: MACアドレス「00:11:1c:f6:yy:xx」のGigEVisionカメラとシリアル番号「00XXYY0」のUSB3Visionカメラ(STC-MCS510U3V)のみ接続します。
    * ``camera_to_connect:["14210003XXYY"]``: ID「14210003XXYY」のカメラのみ接続します.
  * **default_calibration_file** : 指定するキャリブレーションファイルがないカメラを利用するデフォールトキャリブレーションファイルパス。stcamera_nodeが実行中にこのパラメータが変更され、カメラもすでに接続されている場合は、変更を適用するためにカメラの画像取得を無効にして再度有効にして下さい。キャリブレーションファイルの作成と使い方は下記のROSのチュートリアルを参照下さい（英語）：
      * http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
      * http://wiki.ros.org/camera_calibration_parsers
      * http://wiki.ros.org/image_proc
  * *{dev\_CAMERA-NS}*/**calibration\_file** : ネームスペース「*{dev\_CAMERA-NS}*」が持っているカメラ用のキャリブレーションファイルパス。stcamera_nodeが実行中にこのパラメータが変更され、カメラもすでに接続されている場合は、変更を適用するためにカメラの画像取得を無効にして再度有効にして下さい。カメラネームスペースの詳細は次の章をご参照下さい。

### 3.2. カメラの名前空間（ネームスペース）
stcamera_nodeはカメラと接続する際にカメラに対するトピックとサービスを宣言します。接続されたカメラを区別するために、各カメラには独自の名前空間（ネームスペース）があります。カメラのネームスペースは下記のルールで自動的に生成します：

* **camera_to_connect**の設定無又は["all"]の場合、ネームスペースの形式は *dev\_{CAMERA\_ID}*になります。*{CAMERA\_ID}* はカメラのID。接続されたカメラIDが「14210003xxYY」の場合、ネームスペースは「*dev_14210003xxYY*」になります. GigEVisionのMACアドレスなどのカメラIDの英数字以外の文字は、アンダースコアに置き換えられます。
* **camera_to_connect**がCAMERA\_MODEL(SERIAL)或いはCAMERA\_IDの場合、ネームスペースの形式は*dev\_{CAMERA\_MODEL\_SERIAL\_}*或いは *dev\_{CAMERA\_ID}*になります。*{CAMERA\_MODEL\_SERIAL\_}*はCAMERA_MODEL(SERIAL)からの書き換えられた文字列です. *{CAMERA\_ID}* はカメラIDです。英数字以外の文字は、アンダースコアに置き換えられます。

*{dev\_CAMERA-NS}*の値はサービスコール**get_device_list**また**device_connection**トピックでも得られます。

### 3.3. 発行するトピック
stcamera_nodeが発行するトピックは下記の通りです：

No. |　トピック | 説明
:---: | ----------------------- | ----------------------------------------------
1. | **device_connection** | 接続または切断したときに発行されます。
2. | *{dev\_CAMERA-NS}*/**camera_info** | ROSのimage_transport基準です。
3. | *{dev\_CAMERA-NS}*/**chunk** | チャンクデータを取得したときに発行されます。
4. | *{dev\_CAMERA-NS}*/**event** | カメライベントを登録し、登録されたイベントが発生したときに発行されます。
5. | *{dev\_CAMERA-NS}*/**image_raw\*** | 画像データを取得したときに発行されます（ROSのimage_transport基準）。

### 3.4. サービス
stcamera_nodeが提供するサービスは下記の通りです：

（下記に記載するGenICamノードはROSのノードとは全く別の物です）

* **get_device_list** : 検出されたデバイスのリストを取得します。
* **get_gige_ip** : 指定したGigEVisionカメラのIPアドレスを取得します。
* **get_module_list** : 対応するGenICam GenTLモジュール名のリストを取得します（*System*、*Interface*、*LocalDevice*、*RemoteDevice*、*DataStream*）。 モジュール名は特定のGenICamノードにアクセスする際に引数として渡す必要があります。たとえば、readまたはwrite_nodeサービス経由でカメラの*Gain*にアクセスするには、*Gain*がRemoteDeviceモジュールに存在するように指定する必要があります。
* **get_sdk_info** : SentechSDKバージョンとGenTLプロデューサの情報を取得します。
* **set_gige_ip** : 指定したGigEVisionカメラのIPアドレスを設定します。
* *{dev\_CAMERA-NS}*/**enable_chunk** : チャンクを有効または無効にします。
* *{dev\_CAMERA-NS}*/**enable_event_acquisition** : 特定のモジュールのイベント取得を有効または無効にします。
* *{dev\_CAMERA-NS}*/**enable_event_node** : 指定されるGenICamノードのイベントコールバックを有効または無効にします。
* *{dev\_CAMERA-NS}*/**enable_image_acquisition** : 画像取得を有効または無効にします（カメラを接続する際、自動で有効になります）。
* *{dev\_CAMERA-NS}*/**enable_trigger** : 指定されるトリガセレクタとソースに対してトリガを有効または無効にします。
* *{dev\_CAMERA-NS}*/**execute_node** : 指定されるCommandインタフェースタイプを持つ特定のGenicamノードを実行します。
* *{dev\_CAMERA-NS}*/**get_chunk_list** : カメラがサポートするチャンクのリストを取得します。
* *{dev\_CAMERA-NS}*/**get_enum_list** : 指定されるGenICam列挙ノードの列挙のリストを取得します。
* *{dev\_CAMERA-NS}*/**get_event_acquisition_status_list** : 全モジュールのイベント取得のステータスを取得します。
* *{dev\_CAMERA-NS}*/**get_event_node_status_list** : 指定されるモジュールのGenICamノードイベント取得のステータスを取得します。
* *{dev\_CAMERA-NS}*/**get_genicam_node_info** : 指定されるGenICamノードの情報を取得します。
* *{dev\_CAMERA-NS}*/**get_image_acquisition_status** : 画像取得のステータスを取得します。
* *{dev\_CAMERA-NS}*/**get_last_error** : 最後の実行ステータス情報を取得します。エラーコードのリストについては次の章をご参照下さい。
* *{dev\_CAMERA-NS}*/**get_trigger_list** : カメラがサポートしているトリガのリストを取得します。
* *{dev\_CAMERA-NS}*/**image_raw*** : ROSのimage_transportに基づくサービスです.
* *{dev\_CAMERA-NS}*/**read_node** : ノードインタフェースタイプに関係なく、指定されるGenICamノードを読み込みます。
* *{dev\_CAMERA-NS}*/**read_node_bool** : ブール型のインタフェースタイプを持つ指定されるGenICamノードを読み取ります。
* *{dev\_CAMERA-NS}*/**read_node_enum** : 列挙型のインタフェースタイプを持つ指定されるGenICamノードを読み取ります。
* *{dev\_CAMERA-NS}*/**read_node_float** : Floatのインターフェイスタイプを持つ指定されるGenICamノードを読み取ります。
* *{dev\_CAMERA-NS}*/**read_node_int** : 整数のインターフェイスタイプを持つ指定されるGenICamノードを読み取ります。
* *{dev\_CAMERA-NS}*/**read_node_string** : Stringのインターフェイスタイプを持つ指定されるGenICamノードを読み取ります。
* *{dev\_CAMERA-NS}*/**send_soft_trigger** : ソフトウェアトリガを送信します。
* *{dev\_CAMERA-NS}*/**set_camera_info** : ROSのimage_transportに基づくサービスです.
* *{dev\_CAMERA-NS}*/**write_node** : ノードインタフェースタイプに関係なく、指定されるGenICamノードを書き込みます。stcamera_nodeの内部は自動的に渡される文字列値をGenICamノードのインタフェースタイプに変換します。
* *{dev\_CAMERA-NS}*/**write_node_bool** : ブール型のインタフェースタイプを持つ指定されるGenICamノードを書き込みます。
* *{dev\_CAMERA-NS}*/**write_node_enum_int** : 列挙型のインタフェースタイプを持つ指定されるGenICamノードに整数値を書き込みます。
* *{dev\_CAMERA-NS}*/**write_node_enum_str** : 列挙型のインタフェースタイプを持つ指定されるGenICamノードに文字列値（列挙型エントリの記号名）を書き込みます。
* *{dev\_CAMERA-NS}*/**write_node_float** : Floatのインタフェースタイプを持つ指定されるGenICamノードに整数値を書き込みます。
* *{dev\_CAMERA-NS}*/**write_node_int** : 整数のインタフェースタイプを持つ指定されるGenICamノードに整数値を書き込みます。
* *{dev\_CAMERA-NS}*/**write_node_string** : Stringのインタフェースタイプを持つ指定されるGenICamノードに整数値を書き込みます。

注意：

* 機種によって画像取得中にはGenICamノードにアクセスできない場合があります（エラー発生）。その際には画像取得を停止にしてから再度設定やアクセスして下さい。サービスコール**enable_image_acquisition**で画像取得停止ができます。例：<br />
``$ rosservice call /stcamera_node/dev_CAMERA-NS/enable_image_acquisition false``

* IntegerタイプのGenICamノードは設定できる倍数が決まっており、その倍数で設定しないとエラーになります。サービスコール **get_genicam_node_info**で倍数の確認ができます。下記の例の倍数は16 (increment: "16")：<br />
``$ rosservice call /stcamera_node/dev_CAMERA-NS/get_genicam_node_info RemoteDevice Width``
<pre>
name: "Width"
description: "Width of the image provided by the device (in pixels)."
name_space: "Standard"
interface_type: "IInteger"
access_mode: "Read Only"
is_cachable: "Yes"
visibility: "Beginner"
caching_mode: "Write to register, write to cache on read"
is_streamable: True
enum_value_str_list: []
enum_value_int_list: []
current_value: "2448"
min_value: "64"
max_value: "2448"
increment: "16"
unit: ''
child_node_list: []
</pre>


### 3.5. エラーコード
 コード | 説明
 ---: | ---
 < 0 | GenTLエラーコード。[GenICam GenTLドキュメント（英語）]をご参照下さい。
 0 | エラー無.
 30000 | GenICamのジェネリック例外が発生しました。
 30001 | モジュール名が無効です。
 30002 | (GenICam) ノードが無効またはアクセス不能です。
 30003 | イベントがすでに有効です。
 30004 | イベントがすでに無効です。
 30005 | 画像取得がすでに有効です。
 30006 | 画像取得がすでに無効です。
 30007 | チャンクがサポートされていません。
 30008 | チャンク名が無効です。
 30009 | トリガがサポートされていません。
 30010 | トリガ名が無効です。
 30011 | イベントがサポートされていません。
 30012 | イベント名が無効です。

## 4. 使用法
### 4.1. カメラのGainを変更し、image_viewで取得画像を表示する
シンプルな操作と画像表示は下記の通りの流れを参照下さい。

* 全カメラとの接続を設定し、stcamera_nodeを実行します:
``$ roslaunch omronsentech_camera stcamera_node.launch``
* ネームスペースを取得します：
``$ rosservice call /stcamera_node/get_device_list``<br />
<pre>
device_list:
    -
    timestamp:
      secs: 1521005471
      nsecs: 777053680
    device_id: "142100030510"
    device_model: "STC-MCS510U3V"
    device_serial: "XXYYYZZ"
    device_tltype: "U3V"
    device_tl_specific_field: []
    device_tl_specific_value: []
    device_gentl_info:
      vendor: "OMRON_SENTECH"
      version: "1.5"
      producer_version: "1.5.X.X.X"
      full_path: "/opt/sentech/lib/libstgentl.cti"
      tltype: "Mixed"
    device_namespace: "dev_14210XXYYYZZ"
    connected: True
</pre>

* image_viewを実行します：<br />
``$ rosrun image_view image_view image:=/stcamera_node/dev_14210XXYYYZZ/image_raw``、dev_14210XXYYYZZは接続されているカメラのネームスペースです。
* カメラのGain（IFloatタイプ）を100.0に変更する場合は、次のようにGenicam書き込みノード・サービスを使用します：<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/write_node_float RemoteDevice Gain 100.0``<br />又は<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/write_node RemoteDevice Gain "'100.0'"``
* 現在のGain値を読み取るには、次のようにGenICam読み取りノードサービスを使用します：<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/read_node_float RemoteDevice``<br /> 又は <br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/read_node RemoteDevice Gain``
* **get_genicam_node_info** でGainノード情報を確認する場合は：<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/get_genicam_node_info RemoteDevice Gain``<br />
<pre>
name: "Gain"
description: "Controls the selected gain as an absolute physical value. This is an amplification\
	\ factor applied to the video signal."
name_space: "Standard"
interface_type: "IFloat"
access_mode: "Read and Write"
is_cachable: "Yes"
visibility: "Beginner"
caching_mode: "Write to register, write to cache on read"
is_streamable: True
enum_value_str_list: []
enum_value_int_list: []
current_value: "100"
min_value: "0.000000"
max_value: "192.000000"
increment: ''
unit: ''
child_node_list: []
</pre>

### 4.2. Launchファイルスクリプトの事例
Launchファイルスクリプトの例とそのパラメータはパッケージに含まれています。 それらを使用するには、次の手順を参照下さい。

* **stcamera_node.launch**:
このLaunchファイルスクリプトはstcamera_nodeを実行します。検出されたカメラと接続します。 このスクリプトはstcamera_node.yamlにキャリブレーションファイル（オプション）を設定せず、そのまま起動することができます。
* **stcamera_nodelet_manager.launch**:
このLaunchファイルスクリプトは、nodeletバージョンのstcamera_nodeを実行します。検出されたカメラと接続します。このスクリプトはstcamera_nodelet_manager.yamlにキャリブレーションファイル（オプション）を設定せず、そのまま起動することができます。
* **stcamera_node_imageproc.launch**:
このLaunchファイルスクリプトは、接続されたカメラからストリーミングされた画像を表示するstcamera_nodeとimage_viewという2つのノードを実行します。 このスクリプトを起動する際に、stcamera_node_imageproc.launchとstcamera_node_imageproc.yamlにCAMERA_IDまたはCAMERA_MODEL（SERIAL）および名前空間dev_CAMERA-NSの指定が必要となります：
  * stcamera_node_imageproc.launchにある**camera_namespace**：<br /> ``<arg name="camera_namespace" default="dev_CAMERA-NS" />``
  * stcamera_node_imageproc.yamlにある**camera_to_connect**：<br /> ``camera_to_connect: ["CAMERA_ID or CAMERA_MODEL(SERIAL)"]``
  * オプション： stcamera_node_imageproc.yamlにある**default_calibration_file**又は*{dev_CAMERA-NS}*/**calibration_file**：<br />``default_calibration_file: file:///home/mydata/calibration.xml``

### 4.3. ROSノードクライアント事例
exampleフォルダにあるノード「grabber」はシンプルなROSノードクライアント（両方C++とPythonベースがあります）事例として提供します。Grabberは下記の処理を行います：

1. サービス「**get_sdk_info**」を呼び出して、SentechSDKとGenTL情報を取得し、画面に表示します。
2. サービス「**get_device_list**」を呼び出して、接続済みのカメラを検出し、カメラのネームスペースを取得します。
3. サービス「**get_genicam_node_info**」と「**write_node_float**」を呼び出して、カメラのGain値を読み取りと書き込みます。
4. トピック「image_raw」を参照して、十枚まで画像を取得し、画像サイズとエンコード情報を画面に表示します。
5. サービス「**enable_trigger**」を呼び出して、トリガモードを有効にして、十回まで一秒間ごとにサービス「**send_soft_trigger**」を呼び出して、画像を取得します。トリガモードを有効にする際、セレクタはFrameStartを指定し、ソースはSoftwareを指定します。

GigEVisionカメラを利用する場合は、カメラに有効なIPアドレスがあることを確認して下さい。IPアドレスは、サービス「**get_device_list**」または「**get_gige_ip**」を呼び出すことで確認できます。静的IPアドレスは、サービス「**set_gige_ip**」で設定できます。例：<br />
``$ rosservice call /stcamera_node/get_gige_ip "00:11:1c:xx:xx:xx"`` （MACアドレス「00:11:1c:xx:xx:xx」を持っているGigEVisionカメラのIPアドレスを取得する場合）、及び
<br />
``$ rosservice call /stcamera_node/set_gige_ip "00:11:1c:xx:xx:xx" 192.168.y.z 255.255.255.0 0.0.0.0`` （MACアドレス「00:11:1c:xx:xx:xx」を持っているGigEVisionカメラの静的IPアドレスを設定する場合）

## 5. カスタマイズ
stcamera_nodeは、OMRON SENTECHカメラを制御するために、一般的な機能を提供します。特定のカメラに対応するためのカスタマイズが必要な場合は、StCameraInterfaceクラスを継承し、別途のROSノードを作成するか既存のstcamera_node（StCameraNodeクラス）を編集する必要があります。stcamera_node（StCameraNodeクラス）を編集する場合、StCameraNodeクラスのinitializeCamera()関数にあるカメラインターフェイス初期化プロセスを編集する必要があります。
StCameraInterfaceやStCameraNodeクラスの構造については、doxygenドキュメントを参照下さい。

## 6. 質問・フィードバックや不具合の報告
質問・フィードバックやバグが見つかった場合は、support@sentech.co.jpにお問い合わせ下さい。

[GenICam GenApiドキュメント（英語）]:http://www.emva.org/wp-content/uploads/GenICam_Standard_v2_0.pdf
[GenICam GenTLドキュメント（英語）]:http://www.emva.org/wp-content/uploads/GenICam_GenTL_1_5.pdf
