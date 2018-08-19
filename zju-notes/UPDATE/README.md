# Read Me

主要记录每天对代码修改的部分

## 8.17

1. tasks节点中顺风判定的wp范围变大

   修改文件：heading_planning_laylines.py，第93行

   ```python
   if (wp_wind_angle % 360) > 60 and (wp_wind_angle % 360) < 300:
   ```

   

2. 增加功能：如果在tack过程中发现了需要壁障操作，直接跳出tack

   修改文件：heading_planning_laylines.py，第74-78行

   说明：在对continue_tack判断前增加了对wp是否是顺风方向目标的判定

   ```python
   if self.sailing_state != 'normal':
               # A tack/jibe is in progress
               if self.sailing_state == 'switch_to_port_tack':
                   goal_angle = self.nav.beating_angle
                   continue_tack = boat_wind_angle < goal_angle or boat_wind_angle > 120
               else:  # 'switch_to_stbd_tack'
                   goal_angle = -self.nav.beating_angle
                   continue_tack = boat_wind_angle > goal_angle or boat_wind_angle < -120
   
               if (wp_wind_angle % 360) > 60 and (wp_wind_angle % 360) < 300:
                   goal_wind_angle = wp_wind_angle
                   self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
                   state = 'normal'
                   return state, self.nav.wind_angle_to_heading(goal_wind_angle)
   
               if continue_tack:
                   self.debug_pub('dbg_goal_wind_angle', goal_angle)
                   return self.sailing_state, self.nav.wind_angle_to_heading(goal_angle)
               else:
                   # Tack completed
                   self.log('info', 'Finished tack (%s)', self.sailing_state)
                   self.tack_voting.reset(boat_wind_angle > 0)
                   self.sailing_state = 'normal'
   ```

3. 修改default.yaml文件中的hsv颜色范围，识别颜色为橙色

4. 修改camera_detect节点，增加一个topic：detected_percent，记录检测到的比例


## 8.18 更新
1、完善了tasks的重启功能，从挂掉的任务重新开始；
修改文件：tasks 

定义全局变量 chunyu 储存订阅到的'last_task'消息，把 chunyu 赋值给局部变量task，如果是第一次启动，task = chunyu = -1，从头开始执行；否则task != -1, 把记录了last_task 的 task 传给goal_heading_publisher(tasks_runner,task)

``` python
chunyu = -1
def continue_last_task(msg):
    global chunyu 
    chunyu = msg.data

def goal_heading_publisher(tasks_runner,task):

    tasks_runner.start_next_task(task)
        
if __name__ == '__main__':
    try:
        rospy.Subscriber('last_task', Int16, continue_last_task)
        # 必要的延时
        time.sleep(0.1)
        task = chunyu
        rospy.logwarn('task = ' + str(task))
        if task == -1:
            time.sleep(10)
            rospy.logwarn('start our task!')
        #first start sleep for 100s
        else:
            task = task
            rospy.logwarn('start our task again!')
        goal_heading_publisher(tasks_runner,task)

    except rospy.ROSInterruptException:
        pass
```
修改文件：task.py

如果task_num = -1, task刚刚开始，self.task_ix += 1;否则，task_ix = task_num

```python
def start_next_task(self,task_num = -1):
    """
    Step to the next task, making it the active task.
    """
    self.task_ix += 1
    if task_num != -1:
        self.task_ix = task_num
   	if self.task_ix >= len(self.tasks):
       self.log('warning', "Run all tasks, returning to start")
       self.task_ix = -1
    self.active_task = self.tasks[self.task_ix]
```

增加文件：task_continue
