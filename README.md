# 移植手册

每次拉取代码需要做的：
![portable](.\picture\portable.png)

理由：

`SysConfig` 生成的头文件会 `#Include <ti/……>`，其含义是从系统根目录下拉取代码（即 SDK）

理论上可以写脚本来修改，但是暂时先做验证，后续需要频繁协作、迭代时补。