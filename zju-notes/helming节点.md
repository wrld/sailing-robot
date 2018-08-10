# helming节点

文件位置：/home/wangxuexi/sailing-robot/src/sailing_robot/scripts

written by study_wang

## 文件详述

​	按照顺序来看，程序的运行将先执行前面一些初始化的过程，然后中间的函数和类定义我们可以暂时跳过，直接从主函数来看文件。

由于较长，我们分块来讲

#### 文件开头的初始化部分

```python
# Sheet control
WIND      = object()
SHEET_IN  = object()
SHEET_OUT = object()

# Rudder control
PID_GOAL_HEADING  = object()
PID_ANGLE_TO_WIND = object() # set an angle to the wind, 0 being going torward the wind, can be either 0/360 or -180/180
RUDDER_FULL_LEFT  = object() # the boat is going to the left
RUDDER_FULL_RIGHT = object() # the boat is going to the right
```

- 定义了对象，分别用于不同的控制。object()这个具体作用大家可以看看Python，我不是很懂这个语言，所以不班门弄斧了。



```python
# Publishers for rudder and sailsheet control
PUB_RUDDER    = rospy.Publisher('rudder_control', Int16, queue_size=10)  # Use UInt 16 here to minimize the memory use
PUB_SAILSHEET = rospy.Publisher('sailsheet_normalized', Float32, queue_size=10)
PUB_dbg_helming = rospy.Publisher('dbg_helming_procedure', String, queue_size=10)
```

- 发布三个话题，这种PUB_开头的变量记住，后面可能有用。



```python
data = PID_Data()
rudder = rospy.get_param('rudder')
controller = _PID.PID(rudder['control']['Kp'], rudder['control']['Ki'], rudder['control']['Kd'],rudder['maxAngle'], -rudder['maxAngle'])

sail_table_dict   = rospy.get_param('sailsettings/table')
sheet_out_to_jibe = rospy.get_param('sailsettings/sheet_out_to_jibe', False)
sail_table        = SailTable(sail_table_dict)
sail_data         = SailData(sail_table)

TIMEOUT      = rospy.get_param('procedure/timeout')
EXPLORE_COEF = rospy.get_param('procedure/exploration_coefficient')
```

- 简单的说，这里就是一个取出参数，然后生成我们需要的对象的过程了

- 其中：

  1. data是用PID_Data()类为模板初始化，用于记录变量”goal_heading"、“heading"、 "sailing_state"、"tack_rudder"
  2. PID控制器controller是用于计算PID的对象
  3. sail_table和sail_data是用来干啥暂时不明



------

#### 主函数

```python
if __name__ == '__main__':
    try:
        rospy.Subscriber('wind_direction_apparent', Float64, sail_data.update_wind)
        rospy.Subscriber('goal_heading', Float32, data.update_goal_heading)
        rospy.Subscriber('heading', Float32, data.update_heading)
        rospy.Subscriber('sailing_state', String, data.update_sailing_state)
        rospy.Subscriber('tack_rudder', Float32, data.update_tack_rudder)
        
        Helming()
    except rospy.ROSInterruptException:
        pass
```

- 先是订阅了五个话题，"wind_direction_apparent"、”goal_heading"、“heading"、 "sailing_state"、"tack_rudder"。
- 然后调用了类Helming()，下面来仔细看一下这个类的相关内容



------

#### 类Helming()

#### *附带说明类ProcedureHandle()和类ProcedureBase()因为其中全部函数都调用了一遍

```python
class Helming():
    def __init__(self):
        rospy.init_node('helming', anonymous=True)
        self.rate = rospy.Rate(rospy.get_param("config/rate"))

        # Initialisation of the procedure list
        if rospy.get_param('procedure/jibe_to_turn'):
            procedureList = [JibeBasic, TackBasic, TackSheetOut, Tack_IncreaseAngleToWind]
        else:
            procedureList = [TackBasic, TackSheetOut, Tack_IncreaseAngleToWind, JibeBasic]


        self.Proc = ProcedureHandle(procedureList)
        self.Runner()
```

(这个代码粘贴上去看起来就很难受了，大家可以对着自己的代码来看)

- 先来初始化，先是格式化的节点初始化和发布速率设定，然后是“对任务清单的初始化”，任务清单取决于在参数列表中是否有'procedure/jibe_to_turn'
- 然后用定义好的列表来定义对象Proc
- 然后进入下一个循环函数Runner()，这个函数也属于Helming类



```python
    def Runner(self):
        while not rospy.is_shutdown():
            if data.sailing_state == 'normal':
                if self.Proc.ProcedureInProgress():
                    # ending a procedure because it is finished according to the highlevel
                    rospy.logwarn("Procedure success "+ str(self.Proc.currentProcedure)+
                                  " in "+ '{:.2f}'.format(self.Proc.currentProcedure.EnlapsedTime()) + "s")
                    self.Proc.MarkSuccess()

                set_rudder(PID_GOAL_HEADING)
                set_sail(WIND)
            else:
                # Continuing a procedure
                self.runProcedure()

            self.rate.sleep()
```

判断sailing_state是否为"normal"

- 若是，判断是否有任务正在进行，使用变量currentProcedure
  - 若是，则先输出使用的时间和正在进行进程的数量，最后停止这个任务进程，由注释可以看出，由更高层次的判断可以get到这个任务已经可以结束了
  - 然后进入了这个判断就会进行最后两句，将变量传入两个分别控制舵和帆的函数。由于这个地方是针对“normal”的，这里先不多深入，以后再来看
- 若不是，即在”switch“状态，则我们进行函数runProcedure()



```python
    def runProcedure(self):
        if (not self.Proc.ProcedureInProgress()) or (data.sailing_state != self.Proc.currentProcedure.sailing_state):
            # no procedure have been started (=we just decided to swich tack)
            self.Proc.FirstProcedure()
            self.Proc.StartProcedure(data.sailing_state)
            rospy.logwarn("Run procedure     " + str(self.Proc.currentProcedure))

        elif self.Proc.currentProcedure.has_failed():
            rospy.logwarn("Procedure failed  " + str(self.Proc.currentProcedure))

            self.Proc.MarkFailure()
            # if time out we start the next procedure in the list
            self.Proc.NextProcedure()
            self.Proc.StartProcedure(data.sailing_state)
            rospy.logwarn("Run procedure     " + str(self.Proc.currentProcedure))

        # we advance to the next timestep 
        self.Proc.currentProcedure.loop()
```

###### 这里需要强调一下整个节点的思路，Helming节点在这里的想法是有四种走的策略，每个走的策略都会在运动时候计时，显示按照默认的顺序进行，如果第一个动作执行时间超过了TIMEOUT，则进行下一个动作。而整个动作列表的顺序是动态的，每个人的权重会在变化，比如你上次执行这个动作花的时间少则你的权重会小一些（排序的时候权重小的排在前面）

- 还没有进程需要开始，或者说我们刚刚准备开始继续switch tack动作，如果失败的话(运行时间超过了TIMEOUT）就记录失败，并开始执行下一个动作
- 其实下面的StartProcedure（）、MarkFailure（）、MarkSuccess（）、NextProcedure（）、loop（）函数都是非常简单的，这里就不贴出来了，主要函数看一下排序函数。
- FirstProcedure()

```python
    def FirstProcedure(self):
        self.OrderList()
        self.currentProcedureId = 0
    def OrderList(self):
        def get_weight(x):
            if x['TimeList']:
                return np.mean(x['TimeList'])
            else:
                if random.random() < (EXPLORE_COEF / sum([ not xx['TimeList'] for xx in self.ProcedureList])):
                    rospy.logwarn("Random procedure picked")
                    # Add some randomness in choice for untested procedures
                    return 0.1*random.random()
                else:
                    # Just to keep the order given at first if the procedure was not tested yet
                    return TIMEOUT + x['InitPos']*0.01*TIMEOUT

        self.ProcedureList = sorted(self.ProcedureList, key=get_weight)
        # rospy.logwarn(str(self.ProcedureList))
```

###### 这个可以说是Helming节点中，任务排序最核心的部分！

- 看一下权重的计算方式，如果‘TimeList’关键字对应的列表中有值，则对其去平均来当做权重。话句话说就是如果整个动作执行过，则每次执行记录的那个时间的平均值将成为这个动作的权重值。
- 如果还未执行过，则可以看到有两个判断分支。sum([ not xx['TimeList'] for xx in self.ProcedureList])是用来计算列表中的四个字典中的关键字‘TimeList’对应的列表是否为空列表，然后用整这个与随机数进行判断，如果判断为是则输出随机数（其实就相当于直接执行这句话了），如果没有比随机数大则按照权重计算。
- 这个权重项放在这里主要是为了让没有执行过的话也有机会去执行，如果所有的话都执行过，则之后的权重完全是按照这些动作执行过的时间来计算了。



------

#### 类TackBasic、JibeBasic、TackSheetOut和Tack_IncreaseAngleToWind（procedure任务列表中的四种任务）

这里先回顾一下在类Helming中对四种任务进行初始化时候的先后顺序：

```python
# Initialisation of the procedure list
if rospy.get_param('procedure/jibe_to_turn'):
	procedureList = [JibeBasic, TackBasic, TackSheetOut, Tack_IncreaseAngleToWind]
else:
    procedureList = [TackBasic, TackSheetOut, Tack_IncreaseAngleToWind, JibeBasic]
```

- 按照使用遥控器控制船的经验可以猜测，这个参数的的传入是让船优先选择用jibe的方法完成之字形迎风航行，接下来深入看一下每个类的定义来验证一下
  - TackBasic是基础的tack动作，需要右转则右转，需要左转则左转
  - JibeBasic是基础的jibe动作，需要右转的时候用左转一个大弯来实现
  - TackSheetOut是进阶的tack动作，和TackBasic的唯一区别就是，在控制帆上多放一些帆
  - Tack_IncreaseAngleToWind是一个先让船以一个我们设定的角度（略大于最大攻风角，即横向风）前行得到速度，然后再进行转向的tack动作
- 基本上整个Helming节点的除了和硬件连接实现控制的函数没有说明别的都详细的说明了一遍，如果有不懂可以来问我（王学习：）



------

------

------

## 内容概述

这个节点对switch动作有四种方法去实现，然后通过权重排序的方法来让四种方法有序的排列，并按顺序执行，如果一个任务执行了一定时间还没实现则执行下一个，而一般情况下都是执行第一个就能完成动作。每次开始新的一次switch动作我们都会根据之前表现和随机数的一些东西对任务重新进行权重计算和排序来让任务列表更加合适。

四种任务都是有不同的使用环境的。不断尝试里可以去寻找到较好的，但是对环境判断的非常少，也可以认为这样权重出来的任务排序又不是最好的，可以用更多的条件来重新排序实现我们认为最合适当前环境的任务。

