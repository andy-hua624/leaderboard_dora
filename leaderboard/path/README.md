一、目的
将参考轨迹（ENU_Data）替换到指定文件，使 Dora 系统能够跟随参考轨迹运行。
二、文件位置
需替换文件：dora\sensing\gnss_poser\zuobiao.txt
三、操作步骤
准备数据：确认 ENU_Data 格式正确，每行一个坐标点，x/y 坐标用空格分隔
替换数据：
        用文本编辑器打开dora\sensing\gnss_poser\zuobiao.txt
        删除原有内容
        粘贴 ENU_Data 数据
        保存文件