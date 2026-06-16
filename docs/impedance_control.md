# Điều khiển trở kháng (Impedance Control) 
## Định nghĩa Điều khiển trở kháng (Impedance Control) 
là một phương pháp điều khiển trong robot học và cơ điện tử, trong đó hệ thống không điều khiển trực tiếp vị trí hoặc lực một cách độc lập. 

Thay vào đó, bộ điều khiển được thiết kế để điều khiển mối quan hệ động lực học giữa lực tác động và chuyển động của hệ thống. Nói cách khác, khi robot tương tác với môi trường, điều khiển trở kháng quy định cách mà vị trí, vận tốc và gia tốc của robot phản ứng trước các lực bên ngoài. 
## Ý nghĩa vật lý Trở kháng cơ học (Mechanical Impedance) 
là đại lượng biểu thị mức độ chống lại chuyển động của một hệ cơ học khi hệ đó chịu tác động của ngoại lực. Một hệ có trở kháng lớn sẽ khó bị dịch chuyển khi có lực tác động, trong khi một hệ có trở kháng nhỏ sẽ dễ dàng chuyển động hơn dưới cùng một lực tác động. Trong vật lý, điều khiển trở kháng làm cho hệ thống hoạt động tương đương với một hệ cơ học ảo gồm ba thành phần: khối lượng (Mass), lò xo (Spring) và bộ giảm chấn (Damper). Mô hình này thường được gọi là hệ Mass-Spring-Damper. 
# Năng lượng và Tỉ số cản (Damping Ratio) 
## Bản chất năng lượng của các tham số M, B và K 
Trong mô hình trở kháng, robot được mô phỏng như một hệ cơ học ảo gồm khối lượng, lò xo và bộ giảm chấn. Ba tham số M, B và K quyết định cách hệ thống lưu trữ, truyền tải và tiêu tán năng lượng khi tương tác với môi trường. 
### Độ cứng K (Stiffness) – "Bộ nhớ vị trí" và Thế năng 
Độ cứng K tạo ra lực phục hồi tỉ lệ với độ lệch vị trí của hệ thống: $$ F_K = Kx $$ 
Trong đó: 
- $F_K$ là lực đàn hồi sinh ra bởi lò xo ảo. 
- $K$ là độ cứng của hệ thống. 
- $x$ là độ lệch so với vị trí cân bằng. 

Về mặt vật lý, thành phần độ cứng đóng vai trò như một "bộ nhớ vị trí", luôn có xu hướng kéo hệ thống trở về trạng thái cân bằng ban đầu. Khi robot bị dịch chuyển khỏi vị trí mong muốn, năng lượng được tích lũy dưới dạng thế năng đàn hồi: 

$$ E_p = \frac{1}{2}Kx^2 $$
Trong đó: 
- $E_p$ là thế năng đàn hồi được lưu trữ trong lò xo ảo. 
- $K$ là độ cứng của hệ thống. 
- $x$ là độ lệch khỏi vị trí cân bằng. 

Thế năng này càng lớn khi độ lệch vị trí hoặc độ cứng tăng lên. Nếu hệ thống chỉ có thành phần độ cứng $K$ mà không có giảm chấn $B$, lực phục hồi sẽ liên tục chuyển đổi năng lượng giữa động năng và thế năng. Khi robot được kéo lệch khỏi vị trí cân bằng rồi thả ra, nó sẽ tăng tốc trở về vị trí ban đầu. 

Tuy nhiên, do quán tính của hệ thống, robot không dừng lại tại vị trí cân bằng mà tiếp tục chuyển động vượt qua điểm đó. Sau khi vượt qua vị trí cân bằng, động năng lại được chuyển hóa thành thế năng ở phía đối diện. Quá trình trao đổi năng lượng này tiếp tục lặp lại, tạo thành dao động điều hòa quanh vị trí cân bằng. 

Trong điều kiện lý tưởng không tồn tại ma sát hoặc bất kỳ cơ chế tiêu tán năng lượng nào, biên độ dao động sẽ không suy giảm theo thời gian và hệ thống sẽ dao động vĩnh viễn. 

### Khối lượng ảo M (Virtual Inertia) – "Bộ nhớ vận tốc" và Động năng

 Khối lượng ảo $M$ sinh ra lực quán tính chống lại sự thay đổi vận tốc: 
 $$ F_M = M\ddot{x} $$
 Trong đó: 
 - $F_M$ là lực quán tính. 
 - $M$ là khối lượng ảo của hệ thống. 
 - $\ddot{x}$ là gia tốc. 
 
 Không giống như độ cứng $K$, khối lượng $M$ không lưu trữ thế năng mà lưu trữ động năng của hệ thống: 
 $$ E_k = \frac{1}{2}Mv^2 $$
 Trong đó: 
 - $E_k$ là động năng. 
 - $M$ là khối lượng ảo. 
 - $v$ là vận tốc của hệ thống. 
 
 Về mặt vật lý, $M$ đóng vai trò như một "bộ nhớ vận tốc". Một vật thể có khối lượng càng lớn thì càng khó tăng tốc hoặc giảm tốc. 
 
 >Do đó, bất kể lò xo $K$ tạo ra lực phục hồi lớn đến đâu, khối lượng $M$ vẫn làm chậm quá trình thay đổi vận tốc của hệ thống. Điều này khiến chuyển động trở nên mượt hơn và ít bị giật hơn. 
 
 #### Tác dụng bổ trợ Khối lượng ảo còn đóng vai trò như một bộ lọc nhiễu cơ học tự nhiên (low-pass filter). 
 Khi robot chịu một lực tác động đột ngột hoặc xảy ra va chạm mạnh, khối lượng ảo sẽ hấp thụ một phần năng lượng của cú va chạm và ngăn hệ thống thay đổi vận tốc quá nhanh. Nhờ đó, phản ứng của tay máy trở nên ổn định hơn, giảm rung động và góp phần bảo vệ các cơ cấu cơ khí cũng như bộ truyền động. 
 
 ### Giảm chấn B (Damping) – "Bộ triệt tiêu năng lượng" 
 Giảm chấn $B$ sinh ra lực cản tỉ lệ với vận tốc: 
 $$ F_B = B\dot{x} $$
 Trong đó: 
 - $F_B$ là lực giảm chấn. 
 - $B$ là hệ số giảm chấn. 
 - $\dot{x}$ là vận tốc của hệ thống. 
 
 Trong khi $K$ và $M$ chỉ liên tục trao đổi năng lượng giữa thế năng và động năng, thành phần $B$ là cơ chế duy nhất có khả năng loại bỏ năng lượng ra khỏi hệ thống. Công của lực giảm chấn được chuyển hóa thành nhiệt năng hoặc các dạng tổn hao khác, khiến tổng năng lượng cơ học của hệ giảm dần theo thời gian. 
 
 #### Tác dụng bổ trợ:
  Vai trò quan trọng nhất của giảm chấn là triệt tiêu dao động do sự tương tác giữa khối lượng $M$ và độ cứng $K$. Nếu không có giảm chấn, hệ thống sẽ dao động liên tục quanh vị trí cân bằng. Khi tăng giá trị $B$, năng lượng dao động được tiêu tán nhanh hơn, giúp robot ổn định tại vị trí mong muốn trong thời gian ngắn hơn. 
  
  >Do đó, giảm chấn là thành phần quyết định mức độ ổn định và độ "êm" của phản ứng động học trong điều khiển trở kháng.


# Toán học logic:

Người đặt nền móng cho cho thuyết `điều khiển trở kháng`: $Neville Hogan$

**Input:** là chuyển động (vận tốc/vị trí)  
**Output:** Lực

## Phương trình động lực học mục tiêu (Target Dynamics) 
của hệ thống Khối lượng - Lò xo - Giảm chấn ảo được biểu diễn như sau:

$$M_d (\ddot{x} - \ddot{x}_d) + B_d (\dot{x} - \dot{x}_d) + K_d (x - x_d) = F_{ext}$$

Trong đó:
- $x, \dot{x}, \ddot{x}$: Vị trí, vận tốc, gia tốc thực tế của đầu cuối robot (end-effector).
- $x_d, \dot{x}_d, \ddot{x}_d$: Quỹ đạo vị trí, vận tốc, gia tốc mong muốn.
- $M_d$: Ma trận khối lượng ảo (Quán tính mong muốn - Desired Inertia).
- $B_d$: Ma trận giảm chấn ảo (Desired Damping).
- $K_d$: Ma trận độ cứng ảo (Desired Stiffness).
- $F_{ext}$: Lực tương tác từ môi trường bên ngoài lên robot.

## Logic cốt lõi:
- Nếu không có ngoại lực ($F_{ext} = 0$), phương trình trở thành phương trình vi phân thuần nhất. Robot sẽ bám theo quỹ đạo $x_d$ một cách hoàn hảo với sai số tiến về 0.
- Nếu có ngoại lực tác động (ví dụ robot đụng tường), $F_{ext} \neq 0$. Lúc này sai số vị trí $e = x - x_d$ sẽ không bằng 0 nữa. Robot sẽ "lệch" khỏi quỹ đạo mong muốn một khoảng tỷ lệ thuận với lực tác động và nghịch đảo của ma trận độ cứng $K_d$.