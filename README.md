2018/5/25
- 添加手指仿真控制

## 团队日志
2018/05/26
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

