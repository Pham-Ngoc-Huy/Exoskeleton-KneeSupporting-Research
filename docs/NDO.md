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





# Hướng Dẫn Toàn Diện Về Bộ Quan Sát Nhiễu Phi Tuyến (Nonlinear Disturbance Observer - NDOB)
## Từ Lý Thuyết Toán Học Đến Thực Hành Lập Trình C++

---

## Mục Lục
1. [Chương 1: Đặt Vấn Đề Và Mô Hình Hệ Thống](#chuong-1-dat-van-de-va-mo-hinh-he-thong)
2. [Chương 2: Động Học Sai Số Và Sự Ra Đời Của Công Thức Cốt Lõi](#chuong-2-dong-hoc-sai-so-va-su-ra-doi-cua-cong-thuc-cot-loi)
3. [Chương 3: Giải Pháp Đột Phá Của Chen (Biến Phụ Trợ $z$)](#chuong-3-giai-phap-dot-pha-cua-chen-bien-phu-tro-z)
4. [Chương 4: Ví Dụ Thiết Kế Chi Tiết Trên Hệ 1D](#chuong-4-vi-du-thiet-ke-chi-tiet-tren-he-1d)
5. [Chương 5: Hiện Thực Hóa Lập Trình C++ Từ Zero](#chuong-5-hien-thuc-hoa-lap-trinh-c-tu-zero)
6. [Chương 6: Phân Tích Kỹ Thuật Và Hướng Dẫn Khử Nhiễu Thực Tế](#chuong-6-phan-tich-ky-thuat-va-huong-dan-khu-nhieu-thuc-te)

---

<a name="chuong-1-dat-van-de-va-mo-hinh-he-thong"></a>
## Chương 1: Đặt Vấn Đề Và Mô Hình Hệ Thống

### 1.1 Bài toán thực tế
Trong điều khiển tự động, các hệ thống thực tế (như cánh tay robot, máy bay không người lái drone, hay động cơ điện) luôn phải chịu tác động của các thành phần bất định. Chúng bao gồm:
- **Nhiễu ngoài (External Disturbances):** Gió thổi vào drone, lực cản mặt đường thay đổi khi xe chạy.
- **Sai số mô hình (Modeling Uncertainties):** Ma sát phi tuyến trong các khớp cơ khí không đo được chính xác, khối lượng tải thay đổi liên tục.

Nếu chỉ sử dụng bộ điều khiển phản hồi thông thường (như PID), hệ thống sẽ mất thời gian phản ứng sau khi sai số đã xảy ra. Thay vì mua các cảm biến đắt tiền để đo các lực nhiễu này, ta có thể xây dựng một **"Cảm biến phần mềm"** dựa trên toán học nhằm ước lượng trực tiếp nhiễu và bù trừ nó ngay lập tức. Đó chính là lý do ra đời của **Nonlinear Disturbance Observer (NDOB)**.

### 1.2 Mô hình toán học tổng quát
Xét một hệ thống phi tuyến tổng quát bậc 1 (hoặc dạng biểu diễn trạng thái) có dạng:

$$\dot{x} = f(x) + g(x)u + d$$

Trong đó:
*   $x \in \mathbb{R}^n$: Trạng thái của hệ thống (ví dụ: vận tốc, vị trí) mà ta có thể đo đạc được bằng cảm biến thông thường.
*   $\dot{x}$: Đạo hàm theo thời gian của trạng thái (tốc độ thay đổi của trạng thái).
*   $f(x)$: Động học phi tuyến nội tại đã biết của hệ thống (ví dụ: thành phần trọng trường $-g\sin(\theta)$, lực cản không khí bậc hai $-cx^2$).
*   $g(x)$: Ma trận hoặc hệ số đầu vào điều khiển, thể hiện cách tín hiệu điều khiển tác động lên hệ thống.
*   $u$: Tín hiệu điều khiển đầu vào (ví dụ: điện áp cấp vào motor, lực đẩy của động cơ).
*   $d$: Thành phần nhiễu tổng hợp chưa biết (chứa cả nhiễu ngoài và sai số mô hình).

---

<a name="chuong-2-dong-hoc-sai-so-va-su-ra-doi-cua-cong-thuc-cot-loi"></a>
## Chương 2: Động Học Sai Số Và Sự Ra Đời Của Công Thức Cốt Lõi

Để hiểu được bản chất toán học, ta cần đi ngược từ đích đến: **Ta muốn sai số ước lượng phải biến mất theo thời gian.**

### Step 1: Định nghĩa sai số ước lượng (Estimation Error)
Gọi $\hat{d}$ là giá trị nhiễu do bộ quan sát tính toán ra (ước lượng). Sai số giữa nhiễu thực tế $d$ và nhiễu ước lượng $\hat{d}$ được định nghĩa là:

$$e = d - \hat{d}$$

Mục tiêu tối thượng của chúng ta là làm sao cho $e(t) \to 0$ khi thời gian $t \to \infty$.

### Step 2: Thiết kế Động học Sai số mong muốn (Desired Error Dynamics)
Trong lý thuyết điều khiển, cách tốt nhất để bắt một sai số giảm về $0$ một cách ổn định theo hàm mũ (exponentially stable) là ép đạo hàm của nó tỷ lệ nghịch với chính nó thông qua một hệ số dương:

$$\dot{e} = -L(x)e$$

Ở đây, $L(x) > 0$ được gọi là **Observer Gain (Hệ số khuếch đại bộ quan sát)**. Nếu phương trình này được thỏa mãn, nghiệm của sai số sẽ có dạng $e(t) = e(0)e^{-L(x)t}$, tức là luôn hội tụ về $0$.

### Step 3: Giả thuyết "Nhiễu biến thiên chậm" (Slowly Varying Disturbance)
Bây giờ, hãy lấy đạo hàm theo thời gian của định nghĩa sai số ở Step 1:

$$\dot{e} = \dot{d} - \dot{\hat{d}}$$

Trong thực tế, tốc độ tính toán của vi xử lý (kHz) nhanh hơn rất nhiều so với tốc độ thay đổi của các lực nhiễu vật lý. Do đó, ta có thể đưa ra giả thuyết chuẩn trong thiết kế bộ quan sát: **Nhiễu thay đổi rất chậm so với tốc độ hội tụ của bộ quan sát**, tức là:

$$\dot{d} \approx 0$$

Khi đó, phương trình đạo hàm sai số thực tế rút gọn còn:

$$\dot{e} = -\dot{\hat{d}}$$

### Step 4: Ép Động học Thực tế bằng Động học Mong muốn
Ta cân bằng hai biểu thức đạo hàm sai số $\dot{e}$ ở Step 2 và Step 3:

$$-\dot{\hat{d}} = -L(x)e$$

Nhân cả 2 vế với $-1$, ta được:

$$\dot{\hat{d}} = L(x)e$$

### Step 5: Thay thế và Khử nhiễu thực tế $d$
Thay ngược định nghĩa sai số $e = d - \hat{d}$ vào vế phải:

$$\dot{\hat{d}} = L(x)(d - \hat{d})$$
$$\dot{\hat{d}} = -L(x)\hat{d} + L(x)d$$

Nhưng ngặt một nỗi: **Nhiễu thực $d$ là đại lượng ta không biết**, làm sao ta có thể đưa nó vào công thức tính toán của máy tính? 
Câu trả lời nằm ở mô hình toán hệ thống ở Chương 1: 

$$\dot{x} = f(x) + g(x)u + d \implies d = \dot{x} - f(x) - g(x)u$$

Thế biểu thức này của $d$ vào phương trình, ta thu được công thức cốt lõi mà bạn thắc mắc:

$$\dot{\hat{d}} = -L(x)\hat{d} + L(x)(\dot{x} - f(x) - g(x)u)$$

---

<a name="chuong-3-giai-phap-dot-pha-cua-chen-bien-phu-tro-z"></a>
## Chương 3: Giải Pháp Đột Phá Của Chen (Biến Phụ Trợ $z$)

Mặc dù công thức trên rất đẹp về mặt toán học, nó lại gặp một **tử huyệt trong ứng dụng thực tế**: Sự xuất hiện của số hạng $\dot{x}$.

### 3.1 Bẫy đạo hàm số (The Numerical Differentiation Trap)
Trong thực tế, cảm biến chỉ trả về giá trị trạng thái $x$ (ví dụ: cảm biến encoder trả về góc $\theta$). Để có $\dot{x}$, ta phải tính đạo hàm số:

$$\dot{x} \approx \frac{x(t) - x(t-\Delta t)}{\Delta t}$$

Tín hiệu từ cảm biến luôn luôn bị nhiễu độ phân giải hoặc nhiễu trắng đặc trưng. Việc tính đạo hàm trên một tín hiệu nhiễu sẽ khuếch đại biên độ nhiễu lên hàng trăm, hàng ngàn lần, khiến cho tín hiệu ước lượng $\hat{d}$ trở nên rung lắc dữ dội và hoàn toàn mất giá trị sử dụng.

### 3.2 Kỹ thuật đổi biến của GS. Jie Chen
Để giải quyết bài toán này, GS. Jie Chen đã đề xuất đưa vào một **biến phụ trợ $z$** để "hấp thụ" hoàn toàn thành phần $\dot{x}$. Ông định nghĩa $z$ như sau:

$$z = \hat{d} - p(x)$$

Trong đó $p(x)$ là một hàm phi tuyến do kỹ sư tự chọn, sao cho đạo hàm của nó liên quan trực tiếp đến Gain $L(x)$ qua quy tắc đạo hàm hàm hợp (Chain Rule):

$$\dot{p}(x) = \frac{\partial p(x)}{\partial x} \dot{x} = L(x)\dot{x}$$

Nói cách khác, ta chủ động chọn hàm $p(x)$ sao cho: **Đạo hàm riêng của $p(x)$ theo $x$ chính là hệ số khuếch đại $L(x)$**.

### 3.3 Sự triệt tiêu kỳ diệu
Bây giờ, hãy tính đạo hàm theo thời gian của biến phụ trợ $z$:

$$\dot{z} = \dot{\hat{d}} - \dot{p}(x)$$

Thay thế phương trình cốt lõi $\dot{\hat{d}}$ từ Chương 2 và phương trình $\dot{p}(x) = L(x)\dot{x}$ vào:

$$\dot{z} = \left(-L(x)\hat{d} + L(x)\dot{x} - L(x)(f(x) + g(x)u)\right) - L(x)\dot{x}$$

Hãy để ý hai số hạng $+L(x)\dot{x}$ và $-L(x)\dot{x}$. **Chúng triệt tiêu nhau một cách hoàn hảo!**

Phương trình động học của $z$ trở thành:

$$\dot{z} = -L(x)\hat{d} - L(x)(f(x) + g(x)u)$$

Vì $z = \hat{d} - p(x) \implies \hat{d} = z + p(x)$, ta thay $\hat{d}$ vào vế phải để phương trình hoàn toàn độc lập:

$$\dot{z} = -L(x)(z + p(x)) - L(x)(f(x) + g(x)u)$$
$$\dot{z} = -L(x)z - L(x)(p(x) + f(x) + g(x)u)$$

### 3.4 Hệ phương trình NDOB hoàn chỉnh ứng dụng trong lập trình
Cuối cùng, bộ quan sát nhiễu phi tuyến được chạy trong máy tính thông qua hệ 2 phương trình cực kỳ sạch sẽ:

$$\begin{cases} 
\dot{z} = -L(x)z - L(x)\Big(p(x) + f(x) + g(x)u\Big) \\
\hat{d} = z + p(x)
\end{cases}$$

**Ý nghĩa:** Vế phải của phương trình $\dot{z}$ hoàn toàn chỉ chứa các đại lượng đã biết ($f(x), g(x), u$) và đại lượng đo trực tiếp ($x$). Không còn bất kỳ một dấu đạo hàm $\dot{x}$ nào nữa! Máy tính chỉ cần thực hiện phép tích phân số học từ $\dot{z}$ ra $z$, một phép toán cực kỳ mịn và có khả năng lọc nhiễu tự nhiên.

---

<a name="chuong-4-vi-du-thiet-ke-chi-tiet-tren-he-1d"></a>
## Chương 4: Ví Dụ Thiết Kế Chi Tiết Trên Hệ 1D

Để chuẩn bị cho việc lập trình, ta áp dụng lý thuyết trên vào một hệ thống cụ thể.

### 4.1 Định hình bài toán
Xét hệ thống phi tuyến bậc 1:

$$\dot{x} = -x^3 + u + d$$

Đối chiếu với mô hình tổng quát:
*   $f(x) = -x^3$ (Thành phần phi tuyến bậc 3)
*   $g(x) = 1$

### 4.2 Thiết kế bộ quan sát (Chọn các hàm toán học)
1. **Chọn hàm $p(x)$:** Để đơn giản và hiệu quả, ta chọn một hàm tuyến tính theo $x$:
   $$p(x) = \gamma x$$
   *(Trong đó $\gamma$ là một hằng số dương cố định).*
2. **Tính Gain $L(x)$:** Theo định nghĩa, $L(x)$ là đạo hàm của $p(x)$ theo $x$:
   $$L(x) = \frac{\partial p(x)}{\partial x} = \frac{\partial (\gamma x)}{\partial x} = \gamma$$
   Như vậy, trong bài toán này, hệ số khuếch đại bộ quan sát là một hằng số $L(x) = \gamma$.

### 4.3 Phương trình triển khai thuật toán
Thay các hàm vừa chọn vào hệ phương trình NDOB tổng quát ở chương trước:

$$\dot{z} = -\gamma z - \gamma(\gamma x - x^3 + u)$$
$$\hat{d} = z + \gamma x$$

Đây chính là hệ phương trình toán học cuối cùng mà ta sẽ nạp vào code C++.

---

<a name="chuong-5-hien-thuc-hoa-lap-trinh-c-tu-zero"></a>
## Chương 5: Hiện Thực Hóa Lập Trình C++ Từ Zero

Để chuyển đổi toán học thành phần mềm, ta sử dụng **Phương pháp tích phân Euler (Euler Forward Integration)** để tính toán giá trị của $z$ theo từng chu kỳ trích mẫu thời gian $\Delta t$:

$$z(t + \Delta t) = z(t) + \dot{z}(t) \cdot \Delta t$$

### Mã nguồn C++ hoàn chỉnh và chú giải tiếng Việt

```cpp
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>

// Lớp đại diện cho Bộ quan sát nhiễu phi tuyến (NDOB)
class NonlinearDisturbanceObserver {
private:
    double z;      // Biến trạng thái phụ trợ bên trong bộ quan sát
    double gamma;  // Hệ số gain khuếch đại (quyết định tốc độ hội tụ)
    double dt;     // Chu kỳ trích mẫu (Bước thời gian tính toán, giây)

    // Thành phần động học nội tại đã biết của hệ thống: f(x) = -x^3
    double f(double x) const {
        return -std::pow(x, 3);
    }

    // Thành phần tác động của đầu vào điều khiển: g(x) = 1.0
    double g(double x) const {
        return 1.0;
    }

    // Hàm phụ trợ được thiết kế chủ động: p(x) = gamma * x
    double p(double x) const {
        return gamma * x;
    }

public:
    // Hàm khởi tạo bộ quan sát
    NonlinearDisturbanceObserver(double observer_gain, double time_step) 
        : z(0.0), gamma(observer_gain), dt(time_step) {}

    // Cập nhật trạng thái bộ quan sát tại mỗi chu kỳ và trả về giá trị nhiễu ước lượng
    double update(double x, double u) {
        // 1. Tính toán giá trị cấu phần bên trong
        double inner_term = p(x) + f(x) + g(x) * u;
        
        // z_dot = -gamma * z - gamma * (p(x) + f(x) + g(x)*u)
        double z_dot = -gamma * z - gamma * inner_term;
        
        // 2. Tích phân số học Euler để cập nhật trạng thái z cho chu kỳ sau
        z += z_dot * dt;
        
        // 3. Tính toán giá trị nhiễu ước lượng cuối cùng: d_hat = z + p(x)
        double d_hat = z + p(x);
        return d_hat;
    }
};

int main() {
    // ---- THIẾT LẬP THAM SỐ MÔ PHỎNG ----
    const double dt = 0.005;     // Chu kỳ trích mẫu 5ms (Tần số 200Hz)
    const double sim_time = 3.0; // Thời gian mô phỏng tổng cộng 3 giây
    
    // Khởi tạo bộ quan sát với Gain gamma = 15.0 (Tốc độ hội tụ cao)
    NonlinearDisturbanceObserver ndob(15.0, dt);

    // Trạng thái ban đầu của hệ thống vật lý thực tế
    double x = 0.5;              // Trạng thái x ban đầu
    double u = 2.0;              // Tín hiệu điều khiển không đổi cấp vào hệ thống
    double real_disturbance = 0.0; // Nhiễu thực tế tác động lên hệ thống

    std::cout << std::fixed << std::setprecision(4);
    std::cout << std::setw(8) << "Time(s)" 
              << std::setw(12) << "State (x)" 
              << std::setw(15) << "Real Dist (d)" 
              << std::setw(18) << "Est Dist (d_hat)" 
              << std::setw(15) << "Error (e)" << "\n";
    std::cout << std::string(68, '-') << "\n";

    // Vòng lặp thời gian mô phỏng
    for (double t = 0.0; t <= sim_time; t += dt) {
        
        // Giả lập nhiễu động: Kịch bản nhiễu thay đổi đột ngột (Step input)
        // Tại thời điểm t = 1.0s, một lực nhiễu lớn d = 8.0 tác động vào hệ thống
        if (t >= 1.0) {
            real_disturbance = 8.0;
        }

        // Bước 1: Gọi bộ quan sát ước lượng nhiễu dựa trên thông tin trạng thái x và u hiện tại
        double estimated_disturbance = ndob.update(x, u);
        double error = real_disturbance - estimated_disturbance;

        // In kết quả ra màn hình Console định kỳ mỗi 0.3 giây để dễ theo dõi
        if (static_cast<int>(std::round(t / dt)) % 60 == 0) {
            std::cout << std::setw(8) << t 
                      << std::setw(12) << x 
                      << std::setw(15) << real_disturbance 
                      << std::setw(18) << estimated_disturbance 
                      << std::setw(15) << error << "\n";
        }

        // Bước 2: Giả lập động học của hệ thống vật lý thực tế để sinh ra trạng thái x mới
        // Phương trình vi phân thực tế: x_dot = -x^3 + u + d_real
        double x_dot = -std::pow(x, 3) + u + real_disturbance;
        
        // Cập nhật trạng thái vật lý x bằng tích phân Euler (mô phỏng phần cứng thực tế)
        x += x_dot * dt;
    }

    return 0;
}
```

---

<a name="chuong-6-phan-tich-ky-thuat-va-huong-dan-khu-nhieu-thuc-te"></a>
## Chương 6: Phân Tích Kỹ Thuật Và Hướng Dẫn Khử Nhiễu Thực Tế

### 6.1 Nghệ thuật tinh chỉnh hệ số Gain $\gamma$
Việc lựa chọn độ lớn của hằng số $\gamma$ (hoặc ma trận $L(x)$ nói chung) tuân theo một quy luật đánh đổi (Trade-off) kinh điển trong kỹ thuật điều khiển:

1. **Khi chọn $\gamma$ lớn (ví dụ: $\gamma = 50$):**
   * *Ưu điểm:* Bộ quan sát bám theo nhiễu cực kỳ nhanh. Khi nhiễu nhảy bậc đột ngột, thời gian quá độ ngắn, sai số nhanh chóng về $0$.
   * *Nhược điểm:* Hệ thống trở nên cực kỳ nhạy cảm với nhiễu đo lường của cảm biến. Nếu cảm biến có răng cưa nhỏ, tín hiệu $\hat{d}$ đầu ra sẽ bị nhiễu động rất mạnh, có thể gây mất ổn định vòng điều khiển phản hồi.
2. **Khi chọn $\gamma$ nhỏ (ví dụ: $\gamma = 2$):**
   * *Ưu điểm:* Tín hiệu ước lượng cực kỳ mịn, lọc nhiễu cảm biến rất tốt nhờ đặc tính giống một bộ lọc thông thấp (Low-pass filter).
   * *Nhược điểm:* Bộ quan sát phản ứng rất chậm chạp. Khi nhiễu đã tác động phá hoại hệ thống, phải mất một lúc lâu bộ quan sát mới nhận diện được biên độ của nhiễu.

### 6.2 Ứng dụng NDOB vào mạch điều khiển vòng kín (Disturbance Rejection Control)
Mục đích tối thượng của việc tạo ra $\hat{d}$ là để **triệt tiêu nó**. Nếu bạn đang thiết kế một bộ điều khiển như LQR hoặc SMC cho hệ thống, bạn có thể thiết kế tín hiệu điều khiển tổng hợp $u$ dưới dạng:

$$u = u_0 - g^{-1}(x)\hat{d}$$

Trong đó:
*   $u_0$: Là luật điều khiển danh định (ví dụ PID hoặc LQR) được thiết kế lý tưởng khi không có nhiễu.
*   $- g^{-1}(x)\hat{d}$: Là thành phần **bù nhiễu trực tiếp** (Feedforward compensation). 

Khi đưa $u$ này vào hệ thống thực tế $\dot{x} = f(x) + g(x)u + d$, ta có:

$$\dot{x} = f(x) + g(x)\left(u_0 - g^{-1}(x)\hat{d}\right) + d$$
$$\dot{x} = f(x) + g(x)u_0 + \underbrace{d - \hat{d}}_{e \to 0}$$

Hệ thống thực tế lập tức hoạt động chính xác như mô hình lý thuyết không có nhiễu nhờ sai số $e$ nhanh chóng hội tụ về 0. Đây chính là bí quyết giúp các hệ thống cơ điện tử hiện đại hoạt động cực kỳ chính xác bất chấp môi trường khắc nghiệt.