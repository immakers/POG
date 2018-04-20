## 团队日志

2018/4/20（Petori)
> 把机械臂控制部分改写成函数，并使编译通过
> 把`moveit::planning_interface::MoveGroup arm_group("arm")`改为全局变量，方便运动规划调用
> 已找到路径点设置函数，准备明日补充路径插值部分，并补完机械臂运动控制部分


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


## 代码维护分工
- Petori `\kinova_moveit`
