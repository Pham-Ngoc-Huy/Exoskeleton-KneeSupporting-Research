# Non-Linear Disturbance Observer Technique (NDO)
## *References*:
- W.-H. Chen, “Disturbance observer based control for nonlinear systems,”
IEEE/ASME Trans. Mechatronics, vol. 9, no. 4, pp. 706–710, Dec. 2004.
- A. Mohammadi, M. Tavakoli, H. J. Marquez, and F. Hashemzadeh,
“Nonlinear disturbance observer design for robotic manipulators,” Con-
trol Eng. Pract., vol. 21, no. 3, pp. 253–267, Mar. 2013.
- J. Huang, S. Ri, L. Liu, Y. Wang, J. Kim, and G. Pak, “Nonlinear
disturbance observer-based dynamic surface control of mobile wheeled
inverted pendulum,” IEEE Trans. Control Syst. Technol., vol. 23, no. 6,
pp. 2400–2407, Nov. 2015.

## 1. What is NDO ?
>English version:  

Is an advanced control engineering tool used to estimate and compensate for unknown external forces, parametric uncertains, and unmodeled dynamics in systems that do not behave linearly
>Vietnamese version:  

là 1 một công cụ kỹ thuật điều khiển tiên tiến được sử dụng để ước tính và bù trù cho các ngoại lực không xác định được, các bất định tham số, và các động lực chưa được mô hình hóa trong hệ thông không hoạt động tuyến tính

## 2. NDO works  
Instead of reacting to `disturbances` after they affect the system's `output`, an `NDO` acts as an estimator. It uses a mathematical model of the system to compare real-time measurements (like states or output) with the nominal expected behavior. The difference between the two is treated as the disturbance

## 3. Key Types of Observers
### 1. Basic Nonlinear Disturbance Observer (BDO): 

### 


## Theoretical Foundation

For a general affine nonlinear system described by:

$\dot{x} = f(x) + g(x)u +d$  
$y = h(x)$

where:
$x$: is a state vector  
$u$: is the conntrol input  
$d$: is the unknown disturbance

> The obserer design ofther utilizes an auxiliary internal variable vector z, allowing the observer to reconstruct the disturbance d̂ without needing acceleration measurements. Mathematical guarantees of stability (often based on input-to-state stability and Lyapunov functions) are used to prove convergence