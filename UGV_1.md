# UGV_1 — Bức tranh tổng quan về nghiên cứu UGV bánh xe trên địa hình phi cấu trúc

> **Mục đích:** Có thể trình bày mạch lạc bức tranh tổng quan của lĩnh vực, chỉ rõ đề tài của mình nằm ở đâu, liên hệ được các nhóm công trình, và phân biệt *hướng nghiên cứu* với *đóng góp khoa học đã được chứng minh*.  
> **Phạm vi cá nhân:** Wheeled Unmanned Ground Vehicle (UGV) hoạt động trên địa hình gồ ghề/phi cấu trúc; trọng tâm là **terrain-aware path/motion planning**, **terrain traversability**, **rủi ro và bất định**, tiến tới **online replanning** và ràng buộc động học/ổn định xe.  
> **Cập nhật:** 13/10/2026. **Trạng thái:** ghi chú tổng quan và định hướng, chưa phải kết quả thực nghiệm hay khẳng định tính mới.

## 1. Kể bức tranh lớn: ngành này đang giải bài toán gì?

Robot mặt đất tự hành (Autonomous Ground Robot/UGV) phải tự trả lời một chuỗi câu hỏi: **Tôi đang ở đâu? Xung quanh có gì? Bề mặt nào có thể đi qua? Đi bằng đường nào? Và điều khiển bánh xe thế nào để thực sự đi theo đường đó?**

Trong đô thị, xe thường có mặt đường và hình học làn đường tương đối rõ ràng. Ngược lại, ở môi trường **unstructured off-road** — rừng, đồng cỏ, đường đất, đồi dốc, sỏi đá, bùn, công trường, khai khoáng hoặc khu vực sau thiên tai — không thể mặc định rằng một khoảng trống không có vật cản là an toàn để lái qua. Địa hình có thể biến dạng; độ bám thay đổi; cảm biến quan sát không hoàn chỉnh; robot có giới hạn độ dốc, khoảng sáng gầm, bán kính quay, lực kéo và khả năng giữ ổn định.

Vì vậy, **không phải chỉ tìm đường ngắn nhất trong bản đồ 2D**. Vấn đề cốt lõi là **tìm một đường đi mà chiếc UGV cụ thể có khả năng thực hiện trên loại địa hình cụ thể, với mức rủi ro chấp nhận được**.

Survey nền tảng [*Autonomous Driving in Unstructured Off-Road Environments: How Far Have We Come?*](https://doi.org/10.1002/rob.70154) (Journal of Field Robotics, công bố tháng 3/2026) khảo sát **hơn 250 bài báo** và tổ chức bức tranh thành các mảng: dữ liệu, lập bản đồ, định vị/ước lượng tư thế, nhận thức, lập kế hoạch đường đi và lái xe end-to-end. Repository đi kèm duy trì các liên kết tới tài liệu và mã nguồn.

**Một câu tóm tắt hướng đang theo:**

> Tôi nghiên cứu cách sử dụng thông tin hình học, tính chất và mức độ không chắc chắn của địa hình để lập kế hoạch đường đi/chuyển động **an toàn, khả thi và hiệu quả cho UGV bánh xe** trong môi trường off-road, thay vì chỉ tối ưu chiều dài đường.

## 2. Hệ thống tự hành hoàn chỉnh và vị trí của đề tài

Sơ đồ dưới đây là **cách diễn giải theo góc nhìn đề tài**, không phải sao chép nguyên sơ đồ của tác giả survey.

\`\`\`mermaid
flowchart TD
    A["Cảm biến: Camera / LiDAR / IMU / GNSS / odometry"] --> B["Định vị và ước lượng trạng thái"]
    A --> C["Nhận thức: vật cản, bề mặt, hình học"]
    B --> D["Bản đồ địa hình: elevation / point cloud / semantic"]
    C --> D
    D --> E["Traversability: có đi qua được không?"]
    E --> F["Cost / risk / uncertainty map"]
    F --> G["Terrain-aware path / motion planner"]
    B --> G
    G --> H["Theo dõi quỹ đạo và điều khiển"]
    H --> I["UGV tương tác với địa hình"]
    I --> A
    I -. "Phản hồi slip/traction và tái lập kế hoạch" .-> F
\`\`\`

| Thành phần trong survey | Nó giải quyết điều gì? | Quan hệ với đề tài |
|---|---|---|
| **Datasets** | Dữ liệu ảnh, LiDAR, tư thế, nhãn bề mặt, một số dữ liệu động lực học | Nguồn để thử nghiệm; phải kiểm tra có nhãn phục vụ *planning* hay chỉ *perception* |
| **Mapping** | Xây dựng bản đồ hình học/độ cao/ngữ nghĩa | **Đầu vào quan trọng**; giai đoạn đầu có thể giả định đã có bản đồ |
| **Pose estimation / SLAM** | Xác định robot đang ở đâu trên bản đồ | Hệ thống phụ trợ; không phải đóng góp chính trong phạm vi ban đầu |
| **Perception** | Phát hiện vật cản, phân loại bề mặt, ước lượng traversability | **Liên quan trực tiếp**, cung cấp thuộc tính và bất định địa hình |
| **Path planning** | Tính đường đi với chi phí, rủi ro và ràng buộc | **Trọng tâm đóng góp nghiên cứu** |
| **End-to-end driving** | Học chính sách từ cảm biến đến hành động hoặc quỹ đạo | Nhánh đối chiếu, **không phải hướng ưu tiên khởi đầu** |
| **Control** | Bám quỹ đạo, kiểm soát tốc độ và tương tác bánh–đất | Dùng để đánh giá tính khả thi, có thể mở rộng về sau |

### Phân biệt các khái niệm dễ nhầm

- **Free-space / obstacle detection:** nơi nào đang trống, không bị một chướng ngại cứng chặn lại. *Trống* không có nghĩa *đi qua được*.
- **Semantic segmentation:** nhận diện cỏ, đất, sỏi, bùn, nước... Đây là chỉ báo hữu ích, **không trực tiếp đo được** hệ số ma sát, độ lún hoặc xác suất trượt.
- **Traversability assessment:** đánh giá mức có thể vượt qua địa hình, có thể phụ thuộc **robot, trạng thái, tốc độ, hướng di chuyển và thời tiết**.
- **Path planning:** chọn tuyến đi qua không gian/bản đồ, thường ở mức hình học.
- **Motion/trajectory planning:** tạo đường/quỹ đạo có xét hướng xe, bán kính quay, độ cong, tốc độ, thời gian và ràng buộc khả thi.
- **Control:** biến quỹ đạo mong muốn thành lệnh lái/ga/phanh hoặc mô-men, chống sai lệch khi robot chạy thật.

Do đó, điểm nối quan trọng nhất của luận văn là: **từ “bản đồ nhìn thấy gì” sang “xe của tôi nên và có thể đi như thế nào”.**

## 3. Vì sao địa hình phi cấu trúc làm planning khó hơn?

### 3.1. Hình học 3D không thể thu gọn hoàn toàn thành khoảng cách 2D

Hai con đường bằng chiều dài vẫn có thể khác nhau rất lớn về độ dốc, độ gồ ghề, bậc cao, rãnh/hố, khả năng cạ gầm, nghiêng ngang và nguy cơ lật. Cách khởi đầu thực tế là dùng **bản đồ độ cao 2.5D (elevation map)** rồi rút ra slope, roughness, step height và kiểm tra vùng tiếp xúc/footprint của xe. 2.5D thuận tiện nhưng không mô tả tốt vách đứng, hang, địa hình nhiều lớp hay đất biến dạng.

### 3.2. Khả năng đi qua phụ thuộc xe, không chỉ phụ thuộc ô địa hình

Một ô bản đồ không có “traversability tuyệt đối”: UGV bánh nhỏ/bánh lớn, 4WD/2WD, trọng tâm cao/thấp, tốc độ khác nhau có thể đánh giá trái ngược. Cùng một sườn dốc, chạy **dọc dốc** và **ngang dốc** tạo nguy cơ khác nhau. Vì vậy, biểu diễn chi phí hợp lý có thể phụ thuộc cả **vị trí và heading**: \(c(x,y,\psi)\), chứ không nhất thiết chỉ \(c(x,y)\).

### 3.3. Tương tác bánh xe–đất và biến thiên theo thời gian

Bùn, cát, sỏi rời, cỏ ướt, đất cứng tạo ra độ bám, trượt bánh, cản lăn, độ lún khác nhau. Những đại lượng này khó suy ra chính xác từ một ảnh RGB. Kinh nghiệm vừa thu được trong khi xe chạy (slip, tốc độ thực tế, IMU) có thể giúp cập nhật chi phí của địa hình và kích hoạt **replanning**.

### 3.4. Bất định là thuộc tính của bài toán

- **Aleatoric uncertainty:** biến thiên/ồn vốn có trong quá trình, ví dụ tương tác bánh–nền đất khó dự đoán hoàn toàn.
- **Epistemic uncertainty:** mô hình thiếu hiểu biết do ít dữ liệu, cảm biến bị che khuất hoặc gặp loại nền mới.

Cả hai có thể làm planner chọn đường quá mạo hiểm nếu chỉ sử dụng giá trị chi phí trung bình. Tuy nhiên, **thêm một hệ số phạt uncertainty không tự động tạo ra đóng góp mới**: cần chứng minh mô hình rủi ro hợp lý, hiệu chỉnh được và cải thiện an toàn trong đánh giá công bằng.

## 4. Các họ phương pháp đang được nghiên cứu

Repository survey liệt kê đồng thời thuật toán cổ điển, tối ưu hóa, học máy và các phương pháp tính đến risk/reliability. Có thể hiểu chúng theo chức năng sau:

| Họ phương pháp | Ví dụ | Ưu điểm | Điểm cần chú ý trong đề tài |
|---|---|---|---|
| **Graph / grid search** | Dijkstra, A*, D* Lite, AD* | Dễ hiện thực, rõ mục tiêu, baseline tốt | Nếu chi phí chỉ là chiều dài hoặc obstacle thì không hiểu địa hình |
| **Search có ràng buộc hướng xe** | Hybrid A*, state lattice | Tạo tuyến phù hợp bán kính quay/hướng chuyển động hơn A* 2D | Phải thiết kế cost và kiểm tra địa hình theo toàn footprint |
| **Sampling-based** | RRT, RRT*, T-RRT, R2-RRT* | Linh hoạt trong không gian trạng thái lớn và costmap | Hiệu suất/phân phối mẫu, chất lượng nghiệm và ràng buộc rủi ro cần đánh giá |
| **Tối ưu hóa quỹ đạo** | Trajectory optimization, MPC (mở rộng) | Xét động học, độ mượt, ràng buộc | Phụ thuộc mô hình, bài toán có thể khó tối ưu thời gian thực |
| **Learning-based** | RL/DRL, imitation learning, learned costmap | Học từ dữ liệu và tương tác phức tạp | Tổng quát hóa, an toàn, khả năng giải thích, yêu cầu dữ liệu |
| **Risk/reliability-aware** | Risk cost, reliability-based RRT*, chance constraint (định hướng) | Mô tả xác suất thất bại thay vì chỉ đường ngắn | Phải định nghĩa biến cố rủi ro và đánh giá xác suất/calibration |

**Không nên chọn thuật toán chỉ vì đang thịnh hành.** Với luận văn thạc sĩ, A*/Hybrid A* + bản đồ địa hình + rủi ro có thể là điểm xuất phát có khả năng tái lập tốt hơn một mô hình end-to-end lớn.

### Những bài trong repository có liên hệ gần

1. [A Global Path Planning Method for Unmanned Ground Vehicles in Off-Road Environments Based on Mobility Prediction (2022)](https://www.mdpi.com/2075-1702/10/5/375): nối **mobility prediction** với lập kế hoạch toàn cục.
2. [Improved A-Star Algorithm for Long-Distance Off-Road Path Planning Using Terrain Data Map (2021)](https://www.mdpi.com/2220-9964/10/11/785): sử dụng dữ liệu địa hình trong một biến thể A*.
3. [Two-Stage Path Planning for Long-Distance Off-Road Path Planning Based on Terrain Data (2024)](https://www.mdpi.com/2220-9964/13/6/184): hướng lập kế hoạch nhiều giai đoạn với thông tin terrain.
4. [Driving Risk-Aversive Motion Planning in Off-Road Environment (2023)](https://www.sciencedirect.com/science/article/pii/S0957417422024459): trực tiếp liên quan tới **risk-averse motion planning**.
5. [R2-RRT*: Reliability-Based Robust Mission Planning of Off-Road Autonomous Ground Vehicle Under Uncertain Terrain Environment (2021)](https://reliadesign.net/wp-content/uploads/2021/01/09334392.pdf): liên hệ giữa **uncertain terrain**, reliability và RRT*.
6. [Efficient Reliability-Based Path Planning ... Coupling of Surrogate Modeling and RRT (2023)](https://ieeexplore.ieee.org/abstract/document/10194466/): ước lượng độ tin cậy và giảm chi phí tính toán.
7. [A Risk-Aware Planning Framework of UGVs in Off-Road Environment (2024, arXiv)](https://arxiv.org/abs/2402.02457): nền tảng so sánh sát chủ đề **risk-aware UGV planning**.
8. [Learning Risk-Aware Costmaps via Inverse Reinforcement Learning for Off-Road Navigation (2023)](https://ieeexplore.ieee.org/abstract/document/10161268): quan hệ giữa **costmap học từ dữ liệu** và điều hướng.
9. [Generating Robot-Dependent Cost Maps for Off-Road Environments Using Locomotion (2022)](https://ieeexplore.ieee.org/abstract/document/10023567/): luận điểm costmap cần phụ thuộc khả năng vận động của robot.
10. [Gaussian Process-based Traversability Analysis for Terrain Mapless Navigation (2024)](https://arxiv.org/abs/2403.19010): minh họa một cách ghép **traversability uncertainty** và lập kế hoạch RRT*.

> **Cách đọc bài:** không dừng ở tên thuật toán. Với từng bài, ghi cụ thể **input terrain → cách tạo cost/risk → planner → giới hạn xe → dữ liệu/mô phỏng → chỉ số kết quả → mã nguồn**, và câu hỏi *đâu là hạn chế thực sự đã được tác giả chứng minh*. Các mô tả ngắn trên là phân loại theo danh mục nguồn, **không thay thế việc đọc phương pháp và thực nghiệm của bài gốc**.

## 5. Mô hình toán minh họa để hình dung đề tài

Giả sử robot có trạng thái \(s=(x,y,\psi,v)\), nhận bản đồ địa hình \(M\) và lập đường/quỹ đạo \(\pi=\{s_0,\dots,s_T\}\). Một mục tiêu minh họa:

\[
\pi^*=\arg\min_{\pi\in\mathcal{F}}
\left[
w_L L(\pi)
+w_E E(\pi)
+w_R R(\pi)
+w_S S(\pi)
\right],
\]

trong đó:

- \(L\): chiều dài hoặc thời gian đi;
- \(E\): năng lượng/khả năng tiêu hao (nếu có mô hình đủ tin cậy);
- \(R\): đại lượng rủi ro (trượt, lật, mất độ bám, không vượt được...);
- \(S\): độ gồ ghề, độ không mượt hoặc chỉ tiêu ổn định/thoải mái;
- \(\mathcal{F}\): tập quỹ đạo đáp ứng các giới hạn hình học, động học, tránh vật cản và các điều kiện khả thi.

Trong biến thể **chance-constrained**, thay vì chỉ cộng một penalty, có thể phát biểu ràng buộc:

\[
P\!\left(\text{UGV vi phạm điều kiện an toàn trên }\pi\right)\leq\varepsilon.
\]

Đây chỉ là **cấu trúc bài toán tham khảo**, chưa phải thuật toán đã được xác minh. Muốn dùng được cần định nghĩa chính xác “vi phạm”, mô hình xác suất, tương quan lỗi giữa các đoạn đường, cách ước lượng xác suất và ngưỡng \(\varepsilon\). Cần tránh mặc định rằng xác suất an toàn của toàn tuyến bằng tích xác suất từng ô độc lập.

### Pipeline thực nghiệm khả thi cho giai đoạn đầu

**Elevation map + terrain features** → **traversability/chi phí theo robot** → **global A* hoặc Hybrid A*** → **kiểm tra quỹ đạo** → **chạy mô phỏng** → **ghi nhận an toàn, độ dài, thời gian và tỷ lệ thành công**.

Sau khi có baseline tái lập được mới mở rộng theo từng chiều: **uncertainty**, **heading-dependent traversability**, **online map update**, hoặc **slip/rollover constraints**. Cần tách riêng từng biến để biết cải thiện thực sự do đâu.

## 6. Datasets trong survey: có gì, thiếu gì?

| Nguồn dữ liệu | Loại thông tin nổi bật | Dùng vào phần nào? | Lưu ý |
|---|---|---|---|
| [RELLIS-3D](https://github.com/unmannedlab/RELLIS-3D) | Camera/LiDAR, nhãn off-road | Perception, semantic terrain map | Nhãn semantic không mặc nhiên là nhãn rủi ro trượt/lật |
| [GOOSE](https://goose-dataset.de/) | RGB/NIR, LiDAR và semantic | Học/kiểm tra nhận diện địa hình | Phải xây cầu nối từ semantic sang traversability |
| [ORFD](https://github.com/chaytonmin/Off-Road-Freespace-Detection) | RGB/LiDAR, vùng không gian đi được theo benchmark | Freespace detection | Freespace không bảo đảm động lực học khả thi |
| [TartanDrive](https://github.com/castacks/tartan_drive) | Dữ liệu lái off-road phục vụ học dynamics | Tương tác trạng thái–điều khiển, dynamics | Kiểm tra modality, đồng bộ, license và nhãn trước khi chọn |
| [BotanicGarden](https://github.com/robot-pesg/BotanicGarden) | Stereo, LiDAR, ego-motion, mapping/semantic | Mapping, định vị, perception | Không tự có nhãn xác suất hỏng cho mọi loại xe |
| [ORAD-3D](https://github.com/chaytonmin/ORAD-3D-Dataset-For-Off-Road-Autonomous-Driving) | Các tác vụ off-road gồm free-space, occupancy, planning | Tham khảo xu hướng liên kết perception–planning | Kiểm tra task/bản dữ liệu cụ thể trước khi benchmark |

**Điểm đặc biệt quan trọng:** dữ liệu cảm nhận tốt chưa đủ để kết luận planner tốt. Đánh giá một nghiên cứu terrain-aware planning cần có tuyến chạy, đặc tính robot, địa hình, mục tiêu, ground truth rủi ro hoặc mô phỏng vật lý có giới hạn được công bố. Có thể phải **tự tạo một benchmark planning có thể tái lập**, đồng thời trình bày rõ mức độ giống thực tế của môi trường mô phỏng.

## 7. Những khoảng trống *ứng viên* cần kiểm chứng

Các mục sau là **giả thuyết nghiên cứu suy ra khi nối các nhánh trong survey**, *không phải* tuyên bố rằng toàn ngành chưa ai giải quyết:

1. **Nhận thức–lập kế hoạch còn có thể thiếu kết nối đủ thực dụng:** nhiều phương pháp nhận diện semantic/obstacle; chưa chắc chi phí đầu ra đã biểu diễn khả năng di chuyển của **một xe cụ thể** dưới ràng buộc chuyển động.
2. **Chi phí phụ thuộc hướng (directional traversability):** \(c(x,y,\psi)\) có thể phản ánh dốc dọc/ngang và rủi ro nghiêng khác với costmap vô hướng. Phải đối chiếu với nghiên cứu đã làm điều này, không coi đó là ý tưởng chưa từng tồn tại.
3. **Rủi ro và bất định cần kiểm định định lượng:** dự báo nguy cơ trượt/lật có được hiệu chỉnh tốt không? Có cải thiện tỷ lệ thành công dưới distribution shift không? Hay chỉ làm xe đi vòng, tốn thời gian hơn?
4. **Tính khả thi chuyển động:** đường trên lưới có thể đẹp nhưng UGV không thực hiện được do bán kính quay, chướng ngại 3D, giới hạn bám đường hoặc giới hạn ổn định.
5. **Tái lập kế hoạch với phản hồi thực tế:** bản đồ có thể thay đổi sau quan sát hoặc khi xe đo được slip. Cần so sánh lợi ích với chi phí tính toán và độ trễ cập nhật.
6. **Thiếu đánh giá có kiểm soát giữa các nhóm phương pháp:** sự cải thiện có thể do bản đồ chi tiết hơn, heuristic, điều chỉnh tham số hoặc giả định simulator; không nên quy hết cho planner mới.

### Ba câu hỏi nghiên cứu nên mang đi trao đổi với giảng viên hướng dẫn

- **RQ1 — Terrain-aware:** Việc tích hợp đặc trưng độ dốc, độ gồ ghề, độ cao chênh và footprint xe vào costmap cải thiện mức an toàn/khả thi bao nhiêu so với A* chỉ dùng khoảng cách–vật cản?
- **RQ2 — Directional & uncertainty-aware:** Mô hình chi phí phụ thuộc hướng hoặc ràng buộc rủi ro có tạo lợi ích đáng kể so với costmap vô hướng và penalty thông thường trên các dạng địa hình chưa thấy không?
- **RQ3 — Practical online planning:** Khi có quan sát mới hoặc ước lượng slip thay đổi, replanning có giảm tỷ lệ mắc kẹt và vi phạm ràng buộc mà vẫn đáp ứng giới hạn thời gian tính toán không?

**Điều kiện để biến RQ thành đề tài:** rà soát bài 2024–2026, chọn 2–3 baseline mạnh có thể chạy lại, phát hiện một thiếu sót được chứng minh qua thực nghiệm, rồi mới phát biểu đóng góp. Tránh chọn tên “uncertainty-aware A*” là đủ để kết luận có novelty/Q1.

## 8. Cách triển khai một nghiên cứu thạc sĩ hợp lý

| Giai đoạn | Việc làm | Kết quả mong đợi |
|---|---|---|
| 1. Literature mapping | Đọc survey lớn + survey chuyên sâu, lập paper matrix 10–20 công trình sát nhất | Bản đồ state of the art và khoảng trống có bằng chứng |
| 2. Tạo baseline | Bản đồ 2.5D, costmap độ dốc/gồ ghề; A* và Hybrid A* | Planner có thể tái lập, có test unit/scenario |
| 3. Thêm đặc thù xe | Footprint, độ cong, giới hạn dốc, ổn định | Kiểm tra tính khả thi thay vì chỉ tránh obstacle |
| 4. Một đóng góp hẹp | Chọn **một** trong directional cost, calibrated risk, hoặc online replanning | Ablation rõ ràng, không gộp nhiều cải tiến thiếu kiểm chứng |
| 5. Benchmark | Nhiều loại địa hình, mức nhiễu, xe/tham số, seed ngẫu nhiên | Bảng và đồ thị có ý nghĩa thống kê |
| 6. Kiểm chứng tính mới | Đối chiếu các baseline mạnh và paper gần thời điểm nộp | Luận điểm đóng góp/thất bại có thể bảo vệ |

**Các baseline tối thiểu:** (i) shortest-path A* trên obstacle map; (ii) A* + scalar terrain cost; (iii) Hybrid A* + terrain/kinematic constraints; (iv) một baseline risk-aware/reliability-aware sát bài toán; (v) phương pháp đề xuất. Các baseline phải dùng thông tin đầu vào tương đương khi kiểm định tác dụng riêng của planner.

**Các metric nên ghi:** success rate, collision/failure rate, số lần vượt ngưỡng dốc/nghiêng/trượt, path length, traversal time, energy (nếu đo được), planning/replanning latency, tần suất replan, hiệu chuẩn xác suất rủi ro (ví dụ calibration curve/Brier score nếu đưa ra dự báo xác suất), hiệu năng trên địa hình khác huấn luyện.

**Đề xuất kỹ thuật khởi đầu:** MATLAB/Python để prototyping costmap và planner; sau đó ROS 2/Gazebo hoặc một mô phỏng có mô hình bánh–đất được kiểm tra để thử motion/control. Nếu mô phỏng chưa tái hiện slip/sink đáng tin, phải ghi rõ rằng kết quả mới kiểm chứng **geometric traversability**, không khẳng định khả năng đi qua ngoài đời thật.

## 9. Cách tự kể trong 90 giây trước giảng viên

> Hướng nghiên cứu của em nằm trong robot mặt đất tự hành ở môi trường phi cấu trúc, cụ thể là UGV bánh xe chạy trên địa hình gồ ghề. Một hệ thống tự hành hoàn chỉnh gồm cảm biến, định vị, xây dựng bản đồ, nhận thức, lập kế hoạch và điều khiển. Em không đi sâu vào toàn bộ hệ thống mà tập trung vào điểm nối giữa đánh giá địa hình và lập kế hoạch đường đi.
>
> Khó khăn là ngoài đường nhựa, một vùng không có vật cản vẫn có thể khiến xe trượt, lật, mắc kẹt hoặc không leo được dốc. Vì vậy, bài toán không chỉ là tìm đường ngắn mà phải đưa thông tin như độ dốc, độ gồ ghề, hình dạng xe, hướng di chuyển và mức rủi ro vào quá trình chọn đường.
>
> Em dự kiến bắt đầu bằng elevation map và các thuật toán có thể tái lập như A* hoặc Hybrid A*, sau đó khảo sát sâu costmap phụ thuộc xe, phụ thuộc hướng và bất định địa hình. Phần nghiên cứu mới chỉ được chốt sau khi so sánh với các công trình terrain-aware và risk-aware gần đây. Mục tiêu là chứng minh sự cải thiện về tính khả thi và an toàn, chứ không chỉ đường ngắn hay hình minh họa đẹp.

## 10. Điều đã biết, điều chưa được khẳng định

| Đã có cơ sở từ survey và các tài liệu liên quan | Chưa thể khẳng định chỉ từ repository |
|---|---|
| Off-road navigation là hệ thống gồm nhiều khối kết nối | Một planner riêng lẻ giải quyết được toàn bộ tự hành |
| Traversability, terrain cost, reliability/risk planning đều đã có công trình nghiên cứu | Bất kỳ tổ hợp hai từ khóa nào cũng là hướng hoàn toàn mới |
| Có dữ liệu cho perception, mapping và một số dynamics | Các dataset đó đủ nhãn để đánh giá chính xác nguy cơ trượt/lật |
| A*, Hybrid A*, RRT* và learning đều có vị trí trong literature | Thuật toán phức tạp hơn mặc nhiên chạy tốt hơn trên UGV thật |
| Có thể lập các giả thuyết nghiên cứu về directional cost, risk và replanning | Các giả thuyết này đã được chứng minh hoặc đủ tiêu chuẩn Q1 |

## 11. Tài liệu và điểm bắt đầu đọc

### Nguồn tổng quan

1. **Nguồn chính:** Chen Min và cộng sự, [*Autonomous Driving in Unstructured Off-Road Environments: How Far Have We Come?*](https://doi.org/10.1002/rob.70154), *Journal of Field Robotics*, 2026. [Repository cập nhật tài liệu](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Off-Road-Environments) · [arXiv:2410.07701](https://arxiv.org/abs/2410.07701).
2. Wang, N. và cộng sự, [*A Survey on Path Planning for Autonomous Ground Vehicles in Unstructured Environments*](https://doi.org/10.3390/machines12010031), *Machines*, 2024 — **đọc sâu ngay sau survey tổng**, vì tập trung vào traversability, cost estimation và terrain-dependent constraints.
3. [*A Survey on Terrain Traversability Analysis for Autonomous Ground Vehicles: Methods, Sensors, and Challenges*](https://www.journalfieldrobotics.org/Field_Robotics/Volume_2_files/Vol2_49.pdf), 2022 — nền tảng đánh giá khả năng vượt địa hình.
4. [*A Survey of Off-Road Mobile Robots: Slippage Estimation, Robot Control, and Sensing Technology*](https://link.springer.com/article/10.1007/s10846-023-01968-2), 2023 — chiều tương tác bánh–đất, slip và control.

### Trực quan từ repository nguồn

- [Taxonomy của lĩnh vực](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Off-Road-Environments/blob/main/doc/tax.png)
- [Các thành phần hệ thống tự hành](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Off-Road-Environments/blob/main/doc/ad.png)
- [Đặc trưng địa hình phi cấu trúc](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Off-Road-Environments/blob/main/doc/nature.png)
- [Những thách thức chính](https://github.com/chaytonmin/Survey-Autonomous-Driving-in-Unstructured-Off-Road-Environments/blob/main/doc/challenge.png)

### Tài liệu đang có trong dự án cá nhân

- [Báo cáo nền tảng terrain-aware planning (LaTeX)](reports/terrain_ugv_2026/main.tex)
- [Thư mục BibTeX](reports/terrain_ugv_2026/references.bib)
- [Hướng dẫn biên dịch và phạm vi báo cáo](reports/terrain_ugv_2026/README.md)

---

**Kết luận để ghi nhớ:** Hệ thống tự hành trả lời *“nhìn–định vị–hiểu–lên kế hoạch–điều khiển”*. Phần việc của tôi là làm cho **bước lên kế hoạch thật sự hiểu địa hình và chiếc xe**, đặc biệt là **traversability, constraints, risk/uncertainty và khả năng cập nhật khi di chuyển**. Hướng này có cơ sở mạnh trong literature; **đóng góp cụ thể vẫn cần được tìm, tái lập và chứng minh**.