# ros-playground

Package to move turtlebot3 based on defined trajectory function

Robotics subject assignment in University

You can change the equation from generate_position() function in controller_publisher.py

```
    def generate_position(self):
        x = math.cos(self.c)/(1+math.sin(self.c)**2)
        y = 1.4*math.sin(self.c)*math.cos(self.c)/(1+math.sin(self.c)**2)
        # print("C     : ", self.c)
        dest = [x,y]
        return dest
```


![alt text](https://github.com/petrusceles/ros-playground/blob/master/assets/robot_running.gif)
