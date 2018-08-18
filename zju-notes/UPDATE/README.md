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
