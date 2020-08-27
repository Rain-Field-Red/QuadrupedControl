# 基于c++接口的coppelia仿真

参考youtube视频

https://www.youtube.com/watch?v=gQTDW-8lxTI

coppelia接口的使用步骤如下：



## 1、新建控制台程序工程

尝试生成。 成功则编译器工作正常。

## 2、确定配置管理器中选择Release和x86

![image-20200824220524949](.\images\image-20200824220524949.png)

## 3、配置工程属性

工程名上右键单击，选择属性（propeties）

在c/c++选项卡中：

（1）在通用（general）子选项卡中，增加附加包含目录

加入两个目录：remoteApi和include



（2）在Preprocessor子选项卡中增加

NON_MATLAB_PARSING=1

MAX_EXT_API_CONNECTIONS=255



（3）在Precompiled Headers子选项卡中选择non-precompiled headers



在Configuration Properties选项卡中：

在通用（general）子选项卡中

character set选择use multi byte character set

whole program opti选择使用链接时间代码生成（use link time code gen）



## 4、将remoteApi文件夹中的四个文件拷贝到本地

extApi.c

extApi.h

extApiPlatform.c

extApiPlatform.h

拷贝完成后，将这些文件添加到工程中。



## 5、首次编译

首次编译可能会报错。

如果出shared memory的错误，就在预编译选项中增加DO_NOT_USE_SHARED_MEMORY



如果报fopen的错误，就在预编译选项中增加

_CRT_SECURE_NO_WARNINGS



## 6、运行示例

将vrepTest示例代码覆盖原代码。

编译。

运行示例中的coppelia场景。

调试运行程序。









