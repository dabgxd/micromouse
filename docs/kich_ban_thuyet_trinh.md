# Kịch bản thuyết trình – Báo cáo giữa kỳ Hệ thống nhúng

**Đề tài:** Xe tự hành né vật cản · **Thời lượng:** khoảng 14–16 phút + hỏi đáp
**Gợi ý chia người nói** (sửa theo nhóm):

| Phần | Slide | Người nói | Thời gian |
|---|---|---|---|
| Mở đầu, bài toán, cơ khí, linh kiện | 1–4 | Minh | ~3,5 phút |
| Kiến trúc hệ thống, mạch, chân vi điều khiển | 5–7 | Trung | ~3 phút |
| Thuật toán lập quỹ đạo, né vật cản, mô phỏng | 8–11 | Thắng | ~6 phút |
| Firmware và kết quả test thật, kết luận | 12–13 | Thắng (hoặc chia) | ~2,5 phút |

Quy ước: **[Chỉ]** = chỉ tay hoặc con trỏ vào hình trên slide. **→** = câu chuyển sang slide sau.

---

## Slide 1 – Trang bìa (~20 giây)

Em chào thầy và các bạn. Nhóm em gồm ba thành viên: Đỗ Minh, Nguyễn Văn Thắng và Phạm Thành Trung. Hôm nay nhóm em xin trình bày báo cáo giữa kỳ môn Nhập môn Hệ thống nhúng với đề tài **xe tự hành né vật cản**. Bài trình bày gồm ba phần: thứ nhất là bài toán và phần cứng của xe, thứ hai là thuật toán giúp xe tìm đường và né vật cản, thứ ba là firmware và kết quả kiểm thử trên xe thật.

→ Trước hết, em xin nói về bài toán mà xe phải giải quyết.

---

## Slide 2 – Bài toán, mục tiêu và phạm vi (~1 phút)

Xe của nhóm em thi đấu trên một sa hình có sẵn gồm năm ô, mỗi ô sáu mươi xăng-ti-mét, xếp thành hình chữ U. Trên sân có mười bảy hộp cố định đã biết trước vị trí, và khi thi, ban tổ chức đặt thêm **một hộp ngẫu nhiên** ở ô 4 hoặc ô 5 mà xe không được biết trước.

[Chỉ vào khung I] Vì vậy, xe gặp ba thách thức chính. Một là phải đi chính xác qua khúc cua hình chữ U. Hai là phải né được cả các hộp cố định lẫn hộp đặt ngẫu nhiên. Ba là phải tuân thủ luật: bánh xe không được ra ngoài biên và không được đi tắt qua vùng cấm ở giữa sân.

[Chỉ vào khung II] Từ đó nhóm em đặt ra ba mục tiêu. Thứ nhất, xe phải phát hiện vật cản theo thời gian thực. Thứ hai, thuật toán phải vừa nhanh vừa chính xác. Thứ ba, quan trọng nhất là về đích an toàn và chạy sạch, không bị trừ điểm.

→ Để làm được điều đó, đầu tiên là thiết kế cơ khí của xe.

---

## Slide 3 – Thiết kế mô hình 3D (~45 giây)

Đây là mô hình 3D của xe mà nhóm em thiết kế. [Chỉ hình trái] Ở góc nhìn nghiêng, có thể thấy năm cảm biến khoảng cách màu tím được bố trí trên một **khung vòng cung** ở đầu xe, hướng ra năm góc: chín mươi độ trái, bốn mươi lăm độ trái, chính diện, bốn mươi lăm độ phải và chín mươi độ phải. Nhờ vậy xe "nhìn" được cả phía trước lẫn hai bên.

[Chỉ hình phải] Ở góc nhìn từ trên xuống, hai bánh xe đặt ở hai bên, mạch điều khiển và driver động cơ nằm giữa thân xe. Cách bố trí này giữ trọng tâm thấp và nằm gần trục bánh, giúp xe ổn định khi quay tại chỗ. Thân xe dài khoảng 14 xăng-ti-mét, rộng khoảng 9 xăng-ti-mét.

→ Tiếp theo là các linh kiện được dùng trên xe.

---

## Slide 4 – Danh sách linh kiện (~1 phút)

Bộ não của xe là vi điều khiển **STM32F411**, lõi ARM Cortex-M4 chạy 100 MHz, có bộ tính toán số thực nên đủ mạnh cho các phép tính điều khiển và lọc tín hiệu.

Xe dùng **hai động cơ N20 có hộp số kèm encoder**. Encoder đếm được 1400 xung cho mỗi vòng bánh, nghĩa là đo được quãng đường tới khoảng một phần mười mi-li-mét. Hai động cơ được điều khiển bởi mạch cầu **TB6612**, cho phép chạy tiến, lùi và điều chỉnh tốc độ bằng PWM.

Về cảm biến, xe có **năm cảm biến khoảng cách ToF VL53L0X**. Chúng đo khoảng cách bằng thời gian bay của tia laser, đều nối chung một đường I2C. Ngoài ra còn cảm biến quán tính **MPU6050**, nhóm em dùng con quay hồi chuyển của nó để đo góc quay của xe.

Phần còn lại gồm: pin LiPo 7,4 V, mạch hạ áp CN3903 để lấy 5 V cấp cho mạch logic, ST-Link để nạp code, và module Bluetooth **HC-05** để gửi dữ liệu về máy tính trong lúc kiểm thử.

→ Các linh kiện này liên kết với nhau như thế nào, em xin mời bạn Trung trình bày.

---

## Slide 5 – Sơ đồ luồng xử lý (~1 phút)

Đây là sơ đồ luồng dữ liệu của toàn hệ thống. Có thể hình dung nó như một vòng khép kín.

[Chỉ phía trên] Đầu vào là **khối cảm biến khoảng cách** và **khối cảm biến góc quay**. Cả hai gửi dữ liệu về khối xử lý trung tâm qua giao tiếp I2C.

[Chỉ giữa] **Khối xử lý trung tâm** là vi điều khiển STM32. Nó tính toán xem xe đang ở đâu và phải đi tiếp thế nào, rồi gửi lệnh điều khiển gồm tín hiệu PWM và tín hiệu chọn chiều quay xuống **khối driver công suất**. Driver khuếch đại tín hiệu thành dòng điện đủ lớn để quay **khối chấp hành**, tức là hai động cơ.

[Chỉ vòng bên phải] Khi động cơ quay, **encoder** đếm xung và gửi ngược về vi điều khiển. Đây chính là **vòng phản hồi**: xe luôn biết bánh đã quay bao nhiêu để tự điều chỉnh, chứ không phải ra lệnh rồi để mặc.

[Chỉ bên trái] Cuối cùng, **khối giao tiếp không dây** nối với vi điều khiển qua UART, để gửi dữ liệu về máy tính phục vụ kiểm thử và tinh chỉnh.

→ Cụ thể các dây được nối ra sao thì thể hiện trong sơ đồ mạch sau đây.

---

## Slide 6 – Sơ đồ kết nối mạch (~1 phút)

Đây là sơ đồ nguyên lý, nhóm em vẽ bằng KiCad.

[Chỉ phía dưới] Năm cảm biến ToF dùng **chung một bus I2C**, gồm hai dây SDA và SCL. Vấn đề là cả năm cảm biến khi bật lên đều có cùng một địa chỉ mặc định. Vì vậy mỗi cảm biến có thêm một chân **XSHUT** riêng. Lúc khởi động, vi điều khiển bật lần lượt từng cảm biến và đổi địa chỉ cho nó, từ 0x30 đến 0x34. Nhờ vậy năm cảm biến cùng nằm trên một bus mà không bị tranh nhau.

[Chỉ cảm biến MPU] Cảm biến MPU6050 được đặt trên **một bus I2C riêng**, để việc đọc góc quay một nghìn lần mỗi giây không bị chậm lại vì phải chờ năm cảm biến khoảng cách.

[Chỉ TB6612] Driver động cơ nhận ba tín hiệu cho mỗi bánh: một chân PWM điều khiển tốc độ và hai chân chọn chiều quay. [Chỉ HC-05] Module Bluetooth nối với vi điều khiển qua UART. [Chỉ J9, J10] Phần nguồn đi từ pin qua mạch hạ áp xuống 5 V.

→ Ở mức vi điều khiển, các chân được phân bổ như sau.

---

## Slide 7 – Quy hoạch chân STM32F411 (~1 phút)

Nhóm em cấu hình chân bằng phần mềm STM32CubeMX. Nguyên tắc chung là **dùng ngoại vi phần cứng thay vì tự làm bằng phần mềm**, để vi điều khiển rảnh tay cho việc tính toán.

Cụ thể như sau:
- Hai chân **PA8 và PA9** là hai kênh PWM của timer TIM1, điều khiển tốc độ bánh phải và bánh trái.
- **Encoder** bánh trái nối vào timer TIM3, encoder bánh phải nối vào TIM4. Hai timer này đặt ở chế độ đọc encoder, nên **phần cứng tự đếm xung**, CPU không phải xử lý từng xung một.
- **I2C1** ở chân PB8, PB9 dành cho năm ToF. **I2C2** ở chân PB10, PB3 dành cho MPU6050.
- **USART2** ở chân PA2, PA3 nối với Bluetooth, dùng DMA để gửi và nhận dữ liệu mà không làm gián đoạn vòng điều khiển.
- Hai chân **PA13, PA14** dành cho nạp code và gỡ lỗi qua ST-Link.

→ Phần cứng là vậy. Tiếp theo em xin mời bạn Thắng trình bày phần quan trọng nhất: làm sao để xe biết đi đường nào.

---

## Slide 8 – Lập quỹ đạo tối ưu trước khi thi (~1,5 phút)

Vì mười bảy hộp cố định đã biết trước vị trí, nhóm em tính sẵn đường đi tốt nhất **trên máy tính**, rồi nạp kết quả vào xe. Việc tính toán gồm năm bước.

**Bước một**, biến bản đồ thành **trường khoảng cách SDF**. Có thể hình dung nó giống tấm bản đồ có đường đồng mức. Bọn em chia sân thành lưới ô năm mi-li-mét, và mỗi ô ghi sẵn khoảng cách từ ô đó tới vật cản gần nhất. Khi cần kiểm tra xe có đụng box không, máy chỉ việc tra bảng chứ không phải tính lại, nên nhanh hơn rất nhiều lần.

**Bước hai**, dùng thuật toán **Hybrid A\*** để tìm đường. Thuật toán này chỉ sinh ra những đường mà xe thật sự đi được, tức là không cua gắt hơn bán kính mười lăm xăng-ti-mét. Nó còn thử nhiều vị trí xuất phát khác nhau sau vạch, rồi chọn vị trí tốt nhất. Em sẽ giải thích kỹ hơn ở slide sau.

**Bước ba**, **làm mượt bằng L-BFGS**. Mọi người hãy tưởng tượng đường đi là một sợi dây thun đang bị bốn lực kéo: lực kéo cho thẳng, lực chống bẻ cong, lực đẩy ra xa vật cản, và lực giữ gần đường ban đầu. Thả tay ra, sợi dây tự nằm ở vị trí cân bằng. L-BFGS là thuật toán tìm ra vị trí cân bằng đó.

**Bước bốn**, **spline bậc ba và hồ sơ vận tốc**. Spline nối các điểm thành đường cong trơn, giống thanh gỗ mỏng uốn qua các cây đinh. Hồ sơ vận tốc thì giống người lái xe qua đèo: đường thẳng chạy nhanh, trước khúc cua phanh sớm, cua càng gắt càng chậm.

**Bước năm**, xuất toàn bộ thành file `trajectory_data.h` để nạp vào firmware.

[Chỉ hình và các ô số liệu] Kết quả: quãng đường **2,67 mét**, thời gian ước tính khoảng **7,2 giây**, xe luôn cách box ít nhất **36 mi-li-mét**, và qua đủ **24 vạch chia phần** để tính điểm. Đường màu vàng trên hình là chỗ xe chạy nhanh, màu tối là chỗ xe phải chậm lại ở khúc cua.

→ Bây giờ em xin giải thích kỹ hơn về Hybrid A\*, trái tim của phần tìm đường.

---

## Slide 9 – Hybrid A\* (~1,5 phút)

[Chỉ hình trái] Thuật toán A\* thông thường chia sân thành ô vuông và đi từ tâm ô này sang tâm ô kia. Kết quả là đường gấp khúc với góc bốn mươi lăm hoặc chín mươi độ. Xe hai bánh thật không thể bẻ lái đột ngột như vậy.

[Chỉ hình phải] Hybrid A\* giải quyết vấn đề này bằng ba ý tưởng.

**Thứ nhất**, mỗi nút trong thuật toán lưu **tư thế thật** của xe, gồm toạ độ x, y và góc hướng θ, không bị làm tròn về tâm ô. Lưới hai xăng-ti-mét nhân năm độ chỉ dùng để gom các nút gần nhau, tránh tính lặp.

**Thứ hai**, từ mỗi tư thế, thuật toán thử các **cung chuyển động mà xe thật đi được**: đi thẳng, rẽ nhẹ, rẽ gắt. Mỗi cung được kiểm tra va chạm bằng trường khoảng cách SDF đã nói ở slide trước. Các nhánh màu xám trên hình chính là cây tìm kiếm đó, và đường màu xanh là đường cong trơn được chọn.

**Thứ ba**, mỗi nút được chấm điểm bằng công thức **f = g + ε·h**. Trong đó **g** là chi phí thực tế đã đi, có phạt thêm khi cua nhiều. **h** là ước lượng quãng đường còn lại tới đích, tính bằng thuật toán Dijkstra trên lưới. Hệ số **ε** lớn hơn một giúp thuật toán ưu tiên hướng về đích, nên tìm nhanh hơn.

[Chỉ phần hạn chế] Hybrid A\* cũng có hạn chế: đường tìm ra chưa tối ưu tuyệt đối và còn gãy nhẹ, vì vậy mới cần bước làm mượt L-BFGS. Và khi xe bị kẹt, thuật toán được phép thêm **cung đi lùi và xoay tại chỗ**, nhưng bị phạt chi phí cao để chỉ dùng khi thật cần.

→ Đó là đường đi đã lập sẵn. Nhưng khi xe chạy thật và gặp hộp ngẫu nhiên thì sao?

---

## Slide 10 – Bám quỹ đạo và né box ngẫu nhiên (~1,5 phút)

Khi chạy, xe phải tự làm năm việc liên tục.

**Một, định vị bằng EKF.** Xe không có GPS. Mọi người hãy tưởng tượng mình đi trong phòng tối: đếm bước chân thì biết đại khái mình ở đâu nhưng càng đi càng sai, còn chạm tay vào cái bàn quen thuộc thì biết chính xác. Trên xe, encoder và gyro là "đếm bước", còn ToF đo tới các box đã biết là "chạm tay vào bàn". Bộ lọc Kalman mở rộng, EKF, kết hợp cả hai, nên vị trí ước lượng vừa mượt vừa không bị trôi.

**Hai, phát hiện hộp ngẫu nhiên.** Vì xe biết mình đang ở đâu và biết bản đồ, nó tính được ToF *lẽ ra* phải đo được bao nhiêu. Nếu ToF đo **ngắn hơn dự đoán quá năm xăng-ti-mét**, nghĩa là phía trước có một vật không nằm trên bản đồ. Xe liên tục ước lượng tâm của vật đó, và phóng to vùng tránh khi chưa chắc chắn.

**Ba, lập lại đường đi.** Xe dùng Hybrid A\* để tạo một **nhánh né**: rẽ ra khỏi quỹ đạo chính trước hộp, đi vòng qua, rồi nhập lại quỹ đạo chính sau hộp. Nếu không có đường né khi đi tiến, xe được phép **lùi và xoay tại chỗ**.

**Bốn, bám quỹ đạo bằng bộ điều khiển Kanayama.** Nó lái giống một tài xế giỏi: thấy khúc cua thì đánh lái sẵn theo độ cong của đường, đồng thời sửa cả việc lệch sang một bên lẫn việc đầu xe chếch hướng, nên xe không bị lượn qua lượn lại.

**Năm, an toàn.** Nếu cảm biến phía trước báo quá gần hai lần liên tiếp thì xe dừng khẩn cấp, khi đường thông thì tự chạy tiếp.

[Chỉ hình và các ô số liệu] Nhóm em đã thử với **80 vị trí hộp ngẫu nhiên**. Xe về đích cả **80/80** lần, **không lần nào bị trừ điểm**. Các ô màu xanh nhạt là vị trí hộp không chắn đường. Ô xanh đậm là xe né được khi đang đi tiến. Ô tím là xe phải lùi hoặc xoay mới né được.

Em xin nói rõ: đây là **kết quả mô phỏng** bằng Python, có đưa vào trượt bánh, sai số cảm biến và nhiễu, nhưng **chưa kiểm chứng trên xe thật**.

→ Slide sau cho thấy rõ hơn các đường đi mà thuật toán tạo ra.

---

## Slide 11 – Thuật toán luôn sinh quỹ đạo hợp lệ (~1 phút)

Ở slide này, nhóm em chồng cả 80 lần thử lên cùng một bản đồ.

[Chỉ hình trái] Bên trái là **quỹ đạo do thuật toán lập**. Đường đen đậm là quỹ đạo chính, dùng khi hộp không chắn đường. Đường xanh lá là nhánh né khi đi tiến. Đường tím là các nhánh có lùi hoặc xoay, đi vòng lên phía trên rồi quay xuống.

[Chỉ hình phải] Bên phải là **đường xe chạy thật trong mô phỏng**, có nhiễu. Có thể thấy xe bám rất sát các quỹ đạo đã lập.

[Chỉ các ô số liệu] Cụ thể:
- Thuật toán lập được đường cho **cả 80/80** vị trí hộp.
- Khi chạy có nhiễu, xe lệch khỏi quỹ đạo **tối đa khoảng 3 xăng-ti-mét**.
- **Không có lần nào** va chạm hay bánh xe ra ngoài vạch.

*(Ô khoảng cách tới box: chỉ đọc con số này nếu đã chạy lại mô phỏng và khớp với slide. Theo lần chạy gần nhất: quỹ đạo **lập ra** luôn cách box ≥ 25 mm; khi **chạy có nhiễu**, lúc sát nhất còn 21 mm, và thân xe cách biên sân lúc sát nhất 3 mm.)*

→ Phần thuật toán là vậy. Tiếp theo là phần nhóm em đã làm được trên xe thật.

---

## Slide 12 – Kiến trúc firmware và kết quả test phần cứng (~2 phút)

[Chỉ khung 1] Firmware được tổ chức thành **nhiều lớp**, giống các tầng của một toà nhà:
- Tầng dưới cùng là **thư viện HAL** của ST, làm việc trực tiếp với phần cứng.
- Tầng trên là các **driver** cho từng linh kiện: ToF, MPU, encoder và driver động cơ.
- Tiếp theo là lớp **điều khiển bánh xe** gồm bù trước động cơ và PID vị trí.
- Trên nữa là lớp **chuyển động**: đi đúng N xung, giữ vận tốc, chạy thẳng và quay tại chỗ.
- Trên cùng là **ứng dụng**.

Điểm mấu chốt là nhóm em viết **hai ứng dụng dùng chung toàn bộ các lớp bên dưới**. Ứng dụng `hwtest` dùng để kiểm thử từng linh kiện qua một bảng điều khiển trên trình duyệt web. Ứng dụng `main` là firmware thi đấu, đang được phát triển. Nhờ vậy những gì đã test xong được dùng lại nguyên vẹn khi thi.

[Chỉ khung 2] Kết quả kiểm thử **trên xe thật**:
- **Bus I2C** nhận đủ năm ToF và MPU, ổn định qua nhiều lần quét liên tiếp.
- **ToF:** bốn trên năm cảm biến chạy ổn định, sai số từ một đến ba xăng-ti-mét. Riêng cảm biến R45 bị lỗi phần cứng, nhóm em đang thay.
- **MPU6050:** sau khi hiệu chuẩn, độ nhiễu gyro chỉ khoảng 0,07 độ mỗi giây. Firmware còn tự bù độ trôi khi xe đứng yên.
- **Encoder:** đo được 1400 xung một vòng bánh, khớp đúng với lý thuyết, tức 7 xung nhân 4 nhân tỉ số truyền 50.
- **Động cơ và PID:** chạy tiến, lùi, quay đều ổn định. Lúc đầu nhóm em cho bánh chạy đủ số xung rồi phanh, nhưng do quán tính bánh luôn bị vượt quá đích. PID kín vòng đã khắc phục được hiện tượng này.
- **Quay tại chỗ:** kết hợp MPU để đo góc thật và encoder để điều khiển từng bánh, xe dừng đúng góc đặt với sai số rất nhỏ.
- **Bluetooth HC-05:** truyền dữ liệu ổn định ở 38400 baud, gửi số liệu về máy tính 25 lần mỗi giây.

→ Em xin tổng kết lại.

---

## Slide 13 – Cảm ơn (~45 giây)

Tóm lại, đến giữa kỳ nhóm em đã hoàn thành ba việc:
- **Phần cứng:** xe đã lắp hoàn chỉnh, từng linh kiện đều được kiểm thử trên xe thật.
- **Thuật toán:** lập quỹ đạo và né hộp ngẫu nhiên đã kiểm chứng bằng mô phỏng, với kết quả 80/80 lần về đích an toàn.
- **Firmware:** đã có nền tảng driver và điều khiển chuyển động dùng chung.

Trong nửa sau học kỳ, nhóm em sẽ:
- Thay cảm biến R45 bị lỗi.
- Đưa bộ định vị EKF và bộ bám quỹ đạo Kanayama từ mô phỏng vào firmware.
- Chạy thử trọn vòng sân trên xe thật.

Em xin cảm ơn thầy và các bạn đã lắng nghe. Nhóm em rất mong nhận được câu hỏi và góp ý.

---

## Chuẩn bị hỏi đáp

| Câu hỏi có thể gặp | Ý trả lời |
|---|---|
| Vị trí đặt xe ban đầu ở đâu? | Gốc toạ độ ở góc trái vạch xuất phát. Tâm trục bánh ở x = 20 cm (thuật toán chọn sau khi thử khoảng 25 vị trí), y = −9,5 cm (vì mũi xe nằm trước trục bánh 9 cm, phải lùi lại để cả xe nằm sau vạch). |
| Bị reset giữa chừng thì sao? | Có sẵn 24 vị trí đặt lại, mỗi vạch chia phần một vị trí, in thành bảng hình (`recovery_cards.png`): đặt đuôi xe sát vạch, đo dọc vạch đúng số cm ghi trên bảng. |
| Vì sao không dùng A\* thường? | A\* thường cho đường gấp khúc 45° hoặc 90°, xe không bám được. Hybrid A\* chỉ sinh cung mà xe đi được, bán kính cua ≥ 15 cm. |
| EKF đã chạy trên xe chưa? | Chưa. Đã kiểm chứng trong mô phỏng có nhiễu (80/80). Firmware hiện có PID bánh, chạy thẳng giữ hướng bằng gyro, và quay tại chỗ. Đang chuyển EKF vào. |
| Tại sao MPU đặt ở I2C riêng? | Để đọc gyro đều 1000 lần mỗi giây, không phải chờ năm ToF trên bus kia. |
| Năm ToF cùng địa chỉ thì sao? | Mỗi cảm biến có chân XSHUT riêng. Lúc khởi động, firmware bật lần lượt từng cảm biến và đổi địa chỉ cho nó (0x30–0x34). |
| Mất Bluetooth khi đang test thì sao? | Firmware có watchdog: không nhận lệnh quá 0,5 giây thì xe tự dừng. |

**Nguyên tắc:** câu nào chưa làm thì nói rõ *"đã kiểm chứng bằng mô phỏng, chưa chạy trên xe"*. Trả lời trung thực luôn được đánh giá cao hơn trả lời vòng vo.
