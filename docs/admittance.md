# Mô hình Admittance
# 1. Admittance
## Admittance là gì?

**Admittance** (độ dẫn tổng) là **nghịch đảo (reciprocal)** của **Impedance** (trở kháng). Nó biểu diễn mức độ **dễ dàng để dòng điện đi qua** một phần tử hay một mạch điện.
> Units of Admittance ìs `SIEMENS` (S)
**where:**
$SIEMENS = \frac{1}{OHMS}$

Trong miền tần số, admittance được định nghĩa là

```math
Y = \frac{1}{Z} = \frac{I}{V}
```

Trong đó:

- \(Y\): Admittance (đơn vị: Siemens, S)
- \(Z\): Impedance (đơn vị: Ohm, \(\Omega\))
- \(I\): Dòng điện (A)
- \(V\): Điện áp (V)

---

## Impedance

Trở kháng của một mạch điện xoay chiều được biểu diễn dưới dạng

```math
Z = R + jX
```

Trong đó:

- \(R\): Điện trở (Resistance)
- \(X\): Điện kháng (Reactance)
- \(j=\sqrt{-1}\): Đơn vị số phức

---

## Admittance

Tương tự, admittance có thể được biểu diễn dưới dạng

```math
Y = G + jB
```

Trong đó:

- \(G\): Độ dẫn (Conductance) – phần thực của admittance.
- \(B\): Độ cảm dẫn (Susceptance) – phần ảo của admittance.

> **Lưu ý:** Conductance **không phải lúc nào cũng bằng** \(1/R\). Điều này chỉ đúng khi mạch **không có điện kháng** (\(X=0\)).

---

## Chứng minh

Theo định nghĩa,

```math
Y=\frac{1}{Z}
```

Thay biểu thức của trở kháng,

```math
Y=\frac{1}{R+jX}
```

Nhân cả tử và mẫu với số phức liên hợp \((R-jX)\),

```math
Y
=
\frac{1}{R+jX}
\cdot
\frac{R-jX}{R-jX}
```

Ta thu được

```math
Y
=
\frac{R-jX}{R^2+X^2}
```

Tách phần thực và phần ảo,

```math
Y
=
\frac{R}{R^2+X^2}
-
j\frac{X}{R^2+X^2}
```

So sánh với biểu thức

```math
Y = G + jB
```

suy ra

```math
G=\frac{R}{R^2+X^2}
```

```math
B=-\frac{X}{R^2+X^2}
```

---

## Trường hợp đặc biệt

Nếu mạch chỉ chứa điện trở,

```math
X=0
```

thì

```math
Y=\frac{1}{R}
```

khi đó

```math
G=\frac{1}{R},\qquad B=0
```

Điều này cho thấy công thức quen thuộc

```math
G=\frac{1}{R}
```

chỉ đúng đối với **mạch thuần trở**.

---

## Kết luận

- Admittance là nghịch đảo của Impedance:

```math
Y=\frac{1}{Z}
```

- Trong trường hợp tổng quát:

```math
G=\frac{R}{R^2+X^2},\qquad
B=-\frac{X}{R^2+X^2}
```

- Đối với mạch thuần trở (\(X=0\)):

```math
G=\frac{1}{R},\qquad
B=0
```

Nói cách khác, **admittance càng lớn thì dòng điện càng dễ đi qua mạch**, còn **impedance càng lớn thì dòng điện càng khó đi qua mạch**.

# 2. Admittance Model
