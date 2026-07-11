# SMC (Sliding Mode Control)

## 1. SMC là gì?

**Sliding Mode Control (SMC)** là một phương pháp điều khiển phi tuyến tính có những ưu điểm:
- ✅ **Mạnh mẽ (Robust)**: Chống lại được các sai số trong mô hình và nhiễu từ bên ngoài
- ✅ **Đơn giản**: Dễ hiểu và dễ cài đặt thực tế
- ❌ **Nhược điểm**: Chattering (rung lên rung xuống nhanh chóng) → actuators bị hỏng nhanh

---

## 2. Ý tưởng chính của SMC là gì?

Thay vì để hệ chạy tự do, SMC **ràng buộc** quỹ đạo của hệ chỉ được tồn tại trên **một đường/mặt nhất định** gọi là **sliding surface** (mặt trượt).

**Ví dụ đơn giản:**
- Thay vì để quả bóng lăn tự do, SMC tạo ra một "rãnh" và bất kỳ lúc nào quả bóng lệch khỏi rãnh, lực điều khiển sẽ kéo nó trở lại rãnh
- Khi quả bóng ở trên rãnh, nó lăn một cách dễ dự đoán → hệ trở thành **hệ tuyến tính đơn giản**

**Phương trình mặt trượt:**
```math
s = b \cdot x_1 + x_2 = 0
```

Trong đó:
- $s$: hàm chuyển mạch (switching function)
- $x_1$: vị trí (position)
- $x_2$: vận tốc (velocity)
- $b$: tham số điều chỉnh (độ dốc của đường)

Khi $s = 0$, hệ trở thành:
```math
\dot{x_1} = -b \cdot x_1
```

Đây là **hệ bậc nhất tuyến tính** → dễ kiểm soát!

---

## 3. Hàm chuyển mạch (Switching Function) là gì?

### Định nghĩa
Hàm chuyển mạch $s$ là một **hàm bất kỳ** của trạng thái hệ (có thể tuyến tính hoặc phi tuyến). 

**Phương trình tổng quát:**
```math
s = f(x) = C^T \cdot x
```

Trong đó:
- $C$: ma trận hệ số (kích thước: số trạng thái × số input)
- $x$: vector trạng thái của hệ

### Tác dụng
Hàm chuyển mạch định nghĩa **mặt trượt** - nơi hệ sẽ hoạt động như một hệ tuyến tính đơn giản.

> **Hai nhiệm vụ chính của SMC:**
> 1. **Đưa hệ đến mặt trượt** (từ trạng thái ban đầu)
> 2. **Giữ hệ trên mặt trượt** (ổn định trên mặt trượt)


---

## 4. Làm thế nào để SMC phản ứng lại với nhiễu?

**Tình huống:**
- Bình thường (không có nhiễu): Hệ tự di chuyển theo quỹ đạo thiết kế để tiến dần về mặt trượt
- Khi có nhiễu: Quỹ đạo bị lệch khỏi mặt trượt

**Cách SMC phản ứng:**
1. Phát hiện hệ đang lệch khỏi mặt trượt (kiểm tra giá trị $s$)
2. Tạo ra thành phần điều khiển "chuyển mạch" (switching control) để kéo hệ trở lại
3. Khi hệ lại nằm trên mặt trượt → tính ổn định được đảm bảo
4. Đây là lý do SMC **bền vững (robust)** chống lại nhiễu và sai số mô hình

---

## 5. Các bước thiết kế SMC

### Bước 1: Định nghĩa Hàm Chuyển Mạch (Sliding Surface)
Chọn $s = C^T \cdot x$ sao cho khi $s = 0$, hệ có động học mong muốn

### Bước 2: Tìm Luật Điều Khiển để Đạt Mặt Trượt (Reachability)
Đảm bảo là hệ có thể tới được và giữ trên mặt trượt

**Điều kiện Reachability:**
```math
s \cdot \dot{s} < 0
```

Nghĩa là: $s$ và $\dot{s}$ có dấu ngược nhau → hệ luôn hướng về mặt trượt

### Bước 3: Xử lý Chattering bằng Boundary Layer
Thay thế hàm sign bằng hàm mịn hơn để giảm rung lên rung xuống

---

## 6. Ví dụ chi tiết: Hệ Mass-Spring-Damper

### 6.1 Mô hình hệ thống
Hệ phương trình vi phân:
```math
M \cdot \ddot{x} + D \cdot \dot{x} + K \cdot x = u(t)
```

Trong đó:
- $M$: khối lượng
- $D$: hệ số cản
- $K$: độ cứng của lò xo
- $u(t)$: lực điều khiển (input)

### 6.2 Chuyển sang dạng State-Space

Định nghĩa trạng thái:
- $x_1 = x$ (vị trí)
- $x_2 = \dot{x}$ (vận tốc)

Phương trình trạng thái:
```math
\dot{x_1} = x_2
```
```math
\dot{x_2} = -\frac{K}{M} \cdot x_1 - \frac{D}{M} \cdot x_2 + \frac{u(t)}{M}
```

Viết dưới dạng tổng quát:
```math
\dot{x} = f(x) + g(x) \cdot u(t)
```

Trong đó:
- $f(x) = [x_2, -\frac{K}{M} \cdot x_1 - \frac{D}{M} \cdot x_2]^T$
- $g(x) = [0, \frac{1}{M}]^T$

### 6.3 Bước 1: Chọn Hàm Chuyển Mạch

Chọn đơn giản nhất (Linear combination):
```math
s = b \cdot x_1 + x_2
```

Khi $s = 0$:
```math
\dot{x_1} = -b \cdot x_1
```

Đây là **hệ bậc 1 ổn định** (nếu $b > 0$)

### 6.4 Bước 2: Đảm bảo Reachability

Yêu cầu: $s \cdot \dot{s} < 0$

**Reaching Law - Constant Rate:**
```math
\dot{s} = -\eta \cdot \text{sign}(s)
```

hoặc

**Reaching Law - Exponential (phổ biến hơn):**
```math
\dot{s} = -\eta \cdot \text{sign}(s) - K \cdot s
```

Trong đó:
- $\eta$: độ lợi (gain) - kiểm soát tốc độ tiến về mặt trượt
- $K$: tham số exponential - kiểm soát độ ổn định gần mặt trượt

### 6.5 Tính Luật Điều Khiển u(t)

Từ điều kiện Reachability:
```math
\dot{s} = C^T \cdot [\dot{x}] = C^T \cdot [f(x) + g(x) \cdot u(t)]
```

Giải cho $u(t)$:
```math
u(t) = [C^T \cdot g(x)]^{-1} \cdot [-C^T \cdot f(x) + h(s(x))]
```

Trong đó $h(s(x))$ là reaching law được chọn ở bước 2

---

## 7. Giải quyết Chattering bằng Boundary Layer

### Vấn đề Chattering
- Hàm `sign(s)` là không liên tục → khi $s$ thay đổi dấu, lực điều khiển thay đổi đột ngột
- Kết quả: Actuators rung lên rung xuống **rất nhanh** (chattering) → hỏng thiết bị

### Giải pháp: Boundary Layer
Thay `sign(s)` bằng hàm mịn $\Theta(s)$:
```math
\Theta(s) = 
\begin{cases}
1, & s > \Theta  \\
\frac{s}{\Theta}, & |s| \leq \Theta  \\
-1, & s < \Theta
\end{cases}
```
Trong đó $\Theta$ là độ rộng của **boundary layer** (thường là một hằng số nhỏ)

**Kết quả:**
- Lực điều khiển **mịn hơn** khi gần mặt trượt
- Giảm chattering đáng kể
- Hệ vẫn ổn định và bền vững

---

## 8. Tóm tắt quy trình thiết kế SMC

| Bước | Nội dung | Công thức |
|------|---------|----------|
| 1 | Định nghĩa mặt trượt | $s = C^T \cdot x = 0$ |
| 2 | Chọn reaching law | $\dot{s} = -\eta \cdot \text{sign}(s) - K \cdot s$ |
| 3 | Tính luật điều khiển | $u(t) = [C^T \cdot g(x)]^{-1} \cdot [-C^T \cdot f(x) + \dot{s}]$ |
| 4 | Áp dụng boundary layer | $\Theta(s)$ thay cho $\text{sign}(s)$ |

---

## 9. Ưu điểm và nhược điểm

| Ưu điểm | Nhược điểm |
|--------|----------|
| ✅ Bền vững với sai số mô hình | ❌ Chattering (nếu không xử lý) |
| ✅ Chống nhiễu tốt | ❌ Cần biết mô hình hệ (f(x), g(x)) |
| ✅ Dễ cài đặt | ❌ Chọn tham số phức tạp |
| ✅ Hoạt động tốt ở điều kiện khác nhau | ❌ Khó thiết kế cho hệ phức tạp |

## 10. Reference:
[1] https://www.youtube.com/watch?v=RD-2oiwEbDo&t=795s