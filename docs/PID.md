# PID Controller

## 1. Tổng quan

Thư mục `control/PID` chứa một mô-đun bộ điều khiển PID đơn giản với cấu trúc gồm:

- `PID.h` / `PID.cpp`: lớp `PIDController` chịu trách nhiệm tính toán tín hiệu điều khiển.
- `trapezodial_integral.h` / `trapezodial_integral.cpp`: lớp `TrapezodialIntegral` tính thành phần tích phân bằng quy tắc hình thang và chống windup.
- `backward_derivative.h` / `backward_derivative.cpp`: lớp `Derivative` tính thành phần đạo hàm bằng sai phân ngược.
- `main.cpp`: ví dụ chạy thử mô phỏng phản hồi của PID với mô hình đơn giản.

Mục tiêu chính của bộ mã này là triển khai PID theo dạng:

- `P = Kp * error`
- `I = Ki * integral(error)`
- `D = Kd * derivative(error)`

và hạn chế tín hiệu điều khiển bằng `u_max` / `u_min`.

## 2. Cấu trúc và chức năng chính

### 2.1 `PIDController`

File: `PID.h`, `PID.cpp`

#### Thuộc tính

- `Kp`, `Ki`, `Kd`: hệ số điều khiển tỷ lệ, tích phân, đạo hàm.
- `Derivative* derivative`: đối tượng tính đạo hàm.
- `TrapezodialIntegral* integral`: đối tượng tính tích phân.
- `u_max`, `u_min`: giới hạn tín hiệu đầu ra.
- `time_step`: bước thời gian mẫu.

#### Phương thức

- `PIDController(...)`: khởi tạo với gains, thời gian mẫu, và giới hạn đầu ra.
- `~PIDController()`: giải phóng bộ nhớ cho `derivative` và `integral`.
- `double update(double x_desired, double x_actual)`: tính giá trị điều khiển PID từ lỗi hiện tại.
- `void setAntiWindupLimits(double max_limit, double min_limit)`: đặt giới hạn chống tích phân quá tải.
- `void setAntiWindupEnabled(bool enable)`: bật/tắt chống windup.
- `void reset()`: đặt lại trạng thái của bộ tích phân và đạo hàm.
- `double clampOutput(double value) const`: giới hạn giá trị điều khiển trong khoảng `u_min`..`u_max`.

#### Luồng hoạt động trong `update`

1. Tính sai số: `error = x_desired - x_actual`.
2. Tính phần tỷ lệ: `u_p = Kp * error`.
3. Tính phần tích phân: `u_i = Ki * integral->update(error)`.
4. Tính phần đạo hàm: `u_d = Kd * derivative->update(error)`.
5. Tổng hợp: `u_control = u_p + u_i + u_d`.
6. Giới hạn đầu ra: `u_control = clampOutput(u_control)`.

### 2.2 `TrapezodialIntegral`

File: `trapezodial_integral.h`, `trapezodial_integral.cpp`

#### Thuộc tính

- `accumulated_integral`: giá trị tích phân hiện tại.
- `previous_error`: lỗi ở bước trước.
- `max_integral`, `min_integral`: giới hạn chống windup.
- `anti_windup_enabled`: bật/tắt chống windup.
- `dt`: bước thời gian dùng cho tích phân.
- `firstRun`: đánh dấu lần gọi đầu tiên.

#### Phương thức

- `TrapezodialIntegral(double max_limit, double min_limit, double step_time)`: khởi tạo và gán `dt`.
- `double update(double error)`: tính tích phân theo quy tắc hình thang.
- `double getIntegral() const`: trả về giá trị tích phân hiện tại.
- `void reset()`: đặt lại trạng thái tích phân.
- `void setAntiWindupLimits(double max_limit, double min_limit)`: đặt giới hạn tích phân.
- `void setAntiWindupEnabled(bool enable)`: bật/tắt chống windup.
- `double clampIntegral(double value) const`: giới hạn giá trị tích phân.

#### Cách tính

- Nếu lần gọi đầu tiên: lưu `previous_error = error` và trả về `0.0`.
- Nếu không: tính
  - `integration_step = ((error + previous_error) / 2.0) * dt`
  - `accumulated_integral += integration_step`
  - Áp dụng chống windup nếu bật.
  - `previous_error = error`

### 2.3 `Derivative`

File: `backward_derivative.h`, `backward_derivative.cpp`

#### Thuộc tính

- `previousError`: giá trị lỗi trước đó.
- `dt`: bước thời gian mẫu.
- `firstRun`: trạng thái lần gọi đầu.

#### Phương thức

- `Derivative(double sampleTime)`: khởi tạo `dt`.
- `double update(double currentError)`: tính đạo hàm sai phân ngược:
  - `if firstRun => trả về 0 và lưu error`
  - `derivative = (currentError - previousError) / dt`
  - cập nhật `previousError`

## 3. Ví dụ sử dụng trong `main.cpp`

File: `main.cpp`

Mã mẫu tạo một bộ điều khiển PID với:

- `Kp = 2.0`
- `Ki = 0.5`
- `Kd = 0.2`
- `time_step = 0.1`
- `u_max = 100.0`, `u_min = -100.0`

Sau đó mô phỏng đơn giản theo mô hình bậc 1:

- `dx_dt = -x_actual + u_control`
- `x_actual += dx_dt * time_step`

Đây là mô phỏng cơ bản để kiểm tra phản hồi PID, không phải mô hình thực tế của khớp cơ học.

## 4. Ghi chú và kiến thức nền

- PID bao gồm 3 thành phần: P, I, D.
- `TrapezodialIntegral` dùng quy tắc hình thang nên chính xác hơn tích phân chữ nhật.
- `Derivative` dùng sai phân ngược (backward difference), phù hợp khi ta chỉ biết giá trị lỗi hiện tại và giá trị trước đó.
- Giới hạn đầu ra (`u_max`, `u_min`) cần để tránh lệnh quá lớn gây quá tải hệ thống.
- Chống windup (`anti-windup`) quan trọng khi thành phần tích phân tiếp tục tăng khi lỗi vẫn tồn tại.

## 5. Những điểm cần cải thiện

- `main.cpp` hiện dùng mô hình rất đơn giản, cần thay bằng mô hình thực tế của cơ cấu để đánh giá PID chính xác.
- Nếu muốn dùng trên hệ nhúng thật, nên loại bỏ `std::cout` debug trong `update()` và `Derivative::update()` / `TrapezodialIntegral::update()`.
- Có thể bổ sung thêm `reset()` trong `PIDController` trước khi bắt đầu chuỗi điều khiển mới.

## 6. Kết luận

Mô-đun `control/PID` hiện đã có kiến trúc rõ ràng:

- `PIDController` điều phối các thành phần P, I, D.
- `TrapezodialIntegral` xử lý tích phân có `dt` và giới hạn windup.
- `Derivative` xử lý đạo hàm bằng sai phân ngược.

Tài liệu này nên dùng làm cơ sở kiến thức để mở rộng hoặc hiệu chỉnh PID cho ứng dụng exoskeleton.
