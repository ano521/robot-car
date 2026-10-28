模板目录 —— 文件名里的数字就是 find_object_2d 的 id

依据源码 FindObject.cpp:290-308：
  文件名首段数字 = id；但 id 必须是 >0 的正数，
  取名 0.png 会被当成“自动分配 id”，所以模板一律从 1 开始。

    1.png  火龙果        6.png  山竹
    2.png  梨            7.png  西瓜
    3.png  榴莲          8.png  香蕉
    4.png  苹果          9.png  红枣
    5.png  葡萄

图上识别到几号，就让抓取节点抓几号：
    ./ai.sh v0 id4        # 抓 4 号（苹果）

来源：C:\Users\aaa\Downloads\抓取识别图像（墙上）.pdf，
     用 FlateDecode + PNG 预测器还原（每行前面有 1 个 filter 字节）。
对照缩略图：docs/抓取功能/模板对照表.png
