# 节点tasks

文件位置：/home/wangxuexi/sailing-robot/src/sailing_robot/scripts

文件说明：看过教程的同学一定都知道节点的主要作用是发布和接受信息（包括topic、srv、action），但是如果我们不从节点文件开始看则会出现看懂各个函数但是不知道整个工程运行方法的现象。看前建议大家去看一下教程中关于节点文件的说明，里面会说明一下节点文件应该怎么查看，也能减轻大家看node文件的压力。（对不起我每次都废话这么多哈哈哈哈）



## 节点文件说明（很重要）

​	因为这个是我写的第一个节点文件，也是非常重要的一个，所以给大家说明一些需要注意的地方。

​	首先从主函数开始（这个很重要），在执行到发布函数之前，前面的代码相当于一个初始化和订阅话题（只执行一次就可以一直接收这个话题的msg）的过程。

​	而进入了执行函数之后，执行函数里面以这句话为分界：

```python
while not rospy.is_shutdown():
```

前面的只会执行一次，而这句话后面的内容就是一直在循环执行的语句了，也就是说在之后的过程中，节点一直在循环执行这些语句。



## 文件总结

​	因为刚刚写完节点文件说明，所以我们就来即兴的看一下这个文件大致做了说明（具体做了说明下面会一个个详述）。

​	那么我们看看这个文件作用是什么，先创建好一些我们需要的类和静态变量，做好订阅工作（订阅一次之后msg会一直更新），进入发布函数，做好标准化工作（其实相当于发布函数的格式），一直按照设定的频率，根据订阅的话题给我们的msg来计算出state和goal_heading并发布出去。



## 文件详述

### 主函数

```python
rospy.init_node("publish_goal_heading", anonymous=True)
#tasks = rospy.get_param("tasks")
tasks = tasks_from_wps(rospy.get_param("wp"))
nav_options = rospy.get_param("navigation")
nav = Navigation(**nav_options)
tasks_runner = RosTasksRunner(tasks, nav)
```

​	先是初始化节点，开始与ROS master通信。

​	get_param() : 函数就是从参数文件中来取出参数，具体可以自己去看一下。

​	tasks_from_wps()	: 是将我们取出来的参数按照要求变成一个静态的任务列表

​	Navigation() : 用navigation参数文件中的数据套上Navigation类的模板

​	RosTasksRunner() : 同上，是用RosTasksRunner类的模板建一个类



```python
rospy.Subscriber('heading', Float32, nav.update_heading)
rospy.Subscriber('wind_direction_apparent', Float64, nav.update_wind_direction)
rospy.Subscriber('position', NavSatFix, nav.update_position)
rospy.Subscriber('temporary_wp', NavSatFix, insert_waypoint)
rospy.Subscriber('jibe_tack_now', String, jibe_tack_now)
srv = Server(TackVotingConfig, tack_voting_callback)

goal_heading_publisher(tasks_runner)
```

​	前面是接收订阅的话题中的信息，最后一句是调用了上面定义的一个函数，这个函数也是发布函数



### 发布函数

```python
def goal_heading_publisher(tasks_runner):
    pub = rospy.Publisher("goal_heading", Float32, queue_size=10)
    pub_state = rospy.Publisher("sailing_state", String, queue_size=10)
    rate = rospy.Rate(rospy.get_param("config/rate"))

    tasks_runner.start_next_task()
    while not rospy.is_shutdown():
        state, goal_heading = tasks_runner.calculate_state_and_goal()
        pub.publish(goal_heading)
        pub_state.publish(state)
        rate.sleep()
```

​	这个函数中除了标准化的部分，最重要的代码是

 	1. tasks_runner.start_next_task()
		2. state, goal_heading = tasks_runner.calculate_state_and_goal()



​	第二个函数非常的大，我专门写了一个文件说明，文件名就是这个函数名。

​	第一个函数定义如下：

```python
    def start_next_task(self):
        """Step to the next task, making it the active task.
        """
        self.task_ix += 1
        if self.task_ix >= len(self.tasks):
            self.log('warning', "Run all tasks, returning to start")
            self.task_ix = 0

        self.active_task = self.tasks[self.task_ix]
        self.on_temporary_task = False
        endcond = '' # TODO
        self.log('info', "Running task {}: {} with end condition {}".format(
                    self.task_ix, self.active_task.task_kind, '/'.join(endcond)
        ))
        self.active_task.start()
```

​	其实这一段还是比较简单能看懂的，我们设定执行下一个任务，并解除了on_temporary_task这个状态，最后执行函数active_task.start()。需要多说一句，在最开始我们说过节点文件注意事项，这个语句在这里初始化过程中我们先执行了一次是因为

```
class TasksRunner(object):
```

这个类的定义的时候，将变量**self.task_ix**初始化为-1，所以我们要先加1了。

​	对了，active_task.start()定义就是一句pass，翻译过来就是啥都不干（江超大哥如是说道）。

------

​	

### 文件中的其他函数

​	抱着对多看一个函数就对程序的了解更深一点的态度，我们再来看一些这个文件中的剩下几个小函数，当然不看也没什么大关系。

#### 前两个函数

```python
def jibe_tack_now(msg):
    tasks_runner.insert_task({
        'kind': 'jibe_tack_now',
        'action': msg.data,
    })
    
def insert_waypoint(msg):
    tasks_runner.insert_task({
        'kind': 'to_waypoint',
        'waypoint_ll': (msg.latitude, msg.longitude),
        'target_radius': 2.5,     # set default value, hot fix for the force jibing node
        'tack_voting_radius': 1 # set default value, hot fix for the force jibing node
    })
```

​	这两个函数的大致含义是相同的，都是将我们设定的参数传入insert_task这个函数，再来看这个。

```python
def insert_task(self, taskdict):
    """Do a temporary task.

        After completing the temporary task, control will be return to the
        regular task that was active before the temporary task was started.
        """
    # Decrease task_ix so we go back to the current task when this is done.
    if not self.on_temporary_task:
        self.task_ix -= 1
    self.on_temporary_task = True
    self.active_task = self._make_task(taskdict)
    self.active_task.start()
    self.log('info', "Running intermediate task：{}".format(taskdict['kind']))
```

​	就是说，当我们调用这个函数的时候，我们将task工作状态归1，然后启动临时任务状态，主要工作是讲前面两个函数中传入的字典给_make_task()生成任务，这个函数再另外讲。



#### 最后一个函数

```python
def tack_voting_callback(config, level):
    """
    get updates for the dynamic parameters
    """
    radius = config.radius
    samples = config.samples
    threshold = config.threshold
    return config
```

​	这个函数用于服务，就是每次请求的数据进来都需要经过这个函数，关于请求的详细内容大家可以看教程里面了。