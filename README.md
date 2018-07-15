## 团队日志
2018/7/15(丘椿荣)
-添加四元数到targetState.msg
-将物品坐标从相机坐标系转 换到机械臂坐标系下

2018/7/6(丘椿荣)
- 修改取消仿真环境下对kinova手抓的注释

2018/6/22(丘椿荣)
- 修改gazebo中kinect节点发布的话题名称，发现table_basket/urdf/table_kinova.urdf.xacro中文件第211或者213中任意一行修改为/kinect2/qhd/camera_info后kinect2_recognition_node都可以运行，目前尚不清楚改哪个好


2018/6/22(丘椿荣)
- visual_detect_simulate虚拟视觉节点基本完成，初步测试通过
- our_pick_place 对gui界面当前抓取目标和抓取次数的更新提前

2018/6/2(丘椿荣)
- visual_detect_simulate虚拟视觉节点数据收发成功，等待测试our_pack_place节点接收逻辑
- 当前UR控制上的代码注释了，当前为控制KINOVA的

2018/6/2(Petori)
- 解决在gazebo中仿真机械臂末端不向下移动的问题（插值错误导致）
- 补充代码使our_pick_place.cpp完全适用于UR
- 本次上传的代码版本为适用于控制UR实物的版本(所做更改在下一条详述）
- 当使用UR时，需要做出的代码更改如下
 - 将代码中所有的arm_group("arm")改为arm_group("manipulator")
 - 找到setPlacePose函数，根据注释文字，取消一部分代码块的注释
 - 找到pickAndPlace函数，根据注释文字(if we use ur for experiment)，取消一部分代码块的注释
 - 注释掉pickAndPlace函数中关于手爪的定义（两行），和手爪开闭的代码块

2018/6/1(Petori)
- 解决控制UR实物时，机器人末端实际位姿和给定位姿不一致的问题
- 注意！控制UR实物时，需要将our_pick_place.cpp文件中的变量名"arm"改为"manipulator"；且将setPlacePose中的语句块解除注释（调用changePoseForUR实现位姿态转换）
- 目前出现这个问题的原因未知，采取变换位姿的方式能解决这个问题，先这样用着

2018/5/28(Petori)
- 解决了不能再次发送目标点的Bug
- 注释掉了j2s7s300_gazebo_demo.launch中打开rviz的语句
- 问题出在arm_group变量的定义上，该变量不能作为全局变量（会导致程序无法运行），也不能定义在main函数中（调用该变量的函数只能调用一次），只能放在局部函数中定义
- 目前轨迹规划和手指控制在rviz中表现良好，但是在启动gazebo之后仍有大量问题

2018/05/26(YYF)
- 添加仿真手指控制，修改宏Simulation为0即可切换为实物控制

2018/5/8(Petori)
- rostopic发话题测试通过
- 尚存问题： Trajectory message contains waypoints that are not strictly increasing in time.
- 另外：gazebo中模型抖动严重，但不影响规划；机械臂规划和运动速度太慢；

2018/5/4(Petori)
- 设置初始放置位置
- 运动规划结果良好

2018/4/21(Petori)
- 本日改动内容总结如下:
- 删除了"初始位置"的定义，仅定义"放置位置"．运动策略变为－－－机械臂在＂目标位置－放置位置＂间来回运动．
- 修改运动控制函数名为`pickAndPlace`，并对函数内容进行修改
- 增加了两个路径插值函数`pickInterpolation`和`placeInterpolation`
- 在108-110行增加了在获取目标之前，使机械臂运动到"放置位置"的代码
- 补充进手爪控制

2018/4/20（Petori)
- 把机械臂控制部分改写成函数，并使编译通过
- 把`moveit::planning_interface::MoveGroup arm_group("arm")`改为全局变量，方便运动规划调用
- 已找到路径点设置函数，准备明日补充路径插值部分，并补完机械臂运动控制部分


## 必看！！！
- 关于github团队操作以当前repository（POG）master分支下的｀Github团队合作教程.pdf｀为准！


## git基本操作
1.直接更改master分支的内容，并push自己的更改到master分支
- git add -A 准备上传你所有的更改
- git status 查看git给出的更改信息
- git commit -m "提交信息" 给出你的更改说明
- git push　　提交更改　（或者git push origin -u master　设置默认提交更改到master分支）

2.与团队项目同步
- git fetch upstream
- git merge upstream/master

