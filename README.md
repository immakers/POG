## git基本操作
1.直接更改master分支的内容，并push自己的更改到master分支
- git pull origin master 下拉最新版本的repository
- git add -A 准备上传你所有的更改
- git status 查看git给出的更改信息
- git commit -m "提交信息" 给出你的更改说明
- git push　　提交更改　（或者git push origin -u master　设置默认提交更改到master分支）

2.（建议做法）从master分支新建一个dev分支（也可以取别的名字)用于你自己的开发，并push自己的更改到dev分支，最后请求合并dev分支和master分支
- git pull origin master
- git checkout master
- git checkout -b dev
- 进行你自己的代码修改与调试工作
- git add -A
- git status
- git commit
- git push origin dev
- git checkout master
- git merge dev

3.当管理员同意你的merge请求后，会删除原来的dev
- git branch -d dev
- git branch origin --delete dev

4.可能用到的其他操作
- git push [路径/文件名] push单个文件
- git checkout origin master [路径/文件名] 更新单个文件
- git push origin dev 提交更改到dev分支
- git merge dev　合并dev分支到master
- git reset --hard [版本号] 恢复到之前的版本
- git push --force origin master 强制提交到master分支

## 任务分工
