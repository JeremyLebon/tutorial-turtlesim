---
title: Exercises
---

# Exercises

## Exercise 1 
Try to move the second turtle to position `-2 -2  1` (x y theta) by terminal

::: details Solution Exercise 1
```shell
rosservice call /turtle1/teleport_absolute 2 2 0 
```
:::

## Exercise 2 
Try to read out the current position of the second turtle by terminal

::: details Solution Exercise 2

```shell
rostopic echo /turtle2/pose 
```

![](./assets/img_Solution_Q2.png)

:::

## Exercise 3 
Try to control the turtle by command (cmd_vel) to used updaterate should be 10ms by terminal

::: details Solution Exercise 3

```shell
rostopic pub /turtle2/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 

rostopic pub /turtle2/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

::: 

## Exercise 4
Try to visualise the current position in `rqt_plot` by terminal

::: details Solution Exercise 4
```shell
rqt_plot  -e /turtle2/pose/x:y:z
```
:::

## Exercise 5
Try to find the update rate of cmd_vel of the second turtle by terminal

::: details Solution Exercise 5
```shell
rostopic hz /turtle2/cmd_vel 
```
:::
## Exercise 6
Try to change the  color of the pen set by the turtle on the screen. (tip check the `rosservice list`) by terminal

::: details Solution Exercise 6
```shell
rosservice list
rosservice info /turtle2/set_pen 
rosservice call /turtle2/set_pen 125 125 125 2  0
```
:::
Clear the screen 

When you want to CLEAR THE SCREEN 

```bash
rosservice call /clear 
```
