# SMC (Sliding Mode Controller)

## 1. What is SMC ?
Sliding mode controllers is a nonlinear control law that has a few nice properties, such as robustness to uncertainties and disturbances, and is relatively straightforward to implement and tune.  

However, one drawbacks is `chattering` where the system state, and therefore, the actuators trying to maintain the state, jitter back and forth quickly, which might note be desirable

> Sliding mode control : how it is affect trajectory ?

---
## 2. Objective of SMC: 
Sliding mode control try to `constrain` the trajectories so that they can only exist along `a slice` within the plane.
> Why we possibly want this kind of constrained behavior ?

-> Control system now behave like a simple first order linear system.
In one example case, control system can represent like:

$$s = b*x_1 + x_2 = 0$$
where  
$s$: is switching function  
$x_1$: is position  
$x_2$: is velocity  
$b$: is just a parameter that adjust the slope of the line

Then, we can rewrite in `first-order-system`

**where $\dot{x_1}$ = $x_2$**

$$\dot{x_1} = -b*x_1$$

> What is switching function ?
### 2.1. Switching function
---
The switching function can be any function of syste states, `even nonlinear`

However, it is most common just to set it to a linear combination of states, where the number of `rows` of the `matrix c` is the number of states in the system and number of `columns` is the number of `inputs u` into the system.

$$s = f(x) = C^T*x$$

where:  
$C$: is $m_x$ by $n_u$

> **First priority:** keep the states on the line

> **Second job:** drive the states to the sliding surface during which the system is going to behave like a second order system:


**VietNammese version:** Nhiệm vụ thứ hai của bộ điều khiển Sliding Mode là đưa trạng thái hệ tiến về mặt trượt (sliding surface). Trong giai đoạn này, hệ sẽ được thiết kế để có động học giống như một hệ bậc hai mong muốn.

Trong điều kiện bình thường, khi không có nhiễu hay tác động ngoại lực, các trạng thái của hệ sẽ tự di chuyển theo quỹ đạo đã được thiết kế để tiến dần về mặt trượt.

Khi xuất hiện nhiễu hoặc ngoại lực từ bên ngoài, quỹ đạo trạng thái có thể bị lệch khỏi mặt trượt. Nếu nhiễu đủ lớn, trạng thái sẽ bị đẩy ra xa quỹ đạo mong muốn.

Bộ điều khiển Sliding Mode sẽ tạo ra một thành phần điều khiển chuyển mạch (switching control) nhằm chống lại ảnh hưởng của nhiễu và bất định của mô hình. Thành phần này tạo ra lực hoặc mô-men bù thích hợp để kéo trạng thái quay trở lại mặt trượt.

Một khi trạng thái đã nằm trên mặt trượt, hệ sẽ tiếp tục di chuyển dọc theo mặt trượt đến điểm cân bằng mong muốn. Chính cơ chế này tạo nên tính bền vững (robustness) nổi tiếng của Sliding Mode Control đối với nhiễu và sai số mô hình.


### 2.3. How to design SMC?
---
**Work-steps:**
1. Define a sliding surface
2. Reach the sliding surface
3. Handle boundary layer [this can be saturation]

**For example:**  
Mass-Spring-Damper:   

$$M*\ddot{x} + D*\dot{x} + Kx = u(t)$$

we called:

$x_2$ = $\dot{x_1}$

then:

$$\dot{x_2} = \frac{-K}{M} * x_1 - \frac{D}{M} * x_2 + \frac{u(t)}{M}
$$
can be rewrite again as:

$$
\ddot{x_1} = \frac{-K}{M} * x_1 - \frac{D}{M} * \dot{x_1} + \frac{u(t)}{M}
$$
> easier to understand just move the highest order to left hand side

-> come up with:

$$\dot{x} = f(x) + g(x) * u(t) $$

> Now. need to find the switching function

**Reachability condition**

$$s * \dot{s} < 0 $$