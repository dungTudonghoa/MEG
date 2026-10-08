# Terrain-Aware Path Planning for Wheeled UGVs (2026)

Báo cáo tổng quan tài liệu bằng tiếng Việt, định hướng luận văn thạc sĩ. Đây là literature review, chưa có kết quả thực nghiệm.

## Các tệp

- `main.tex`: báo cáo LaTeX, phương trình, sơ đồ TikZ, bảng so sánh và định hướng nghiên cứu.
- `references.bib`: thư mục tham khảo (các bài nền tảng và nghiên cứu gần đây).
- `latexmkrc`: hỗ trợ biên dịch XeLaTeX/Biber.
- `.gitignore`: bỏ qua tệp build.

## Biên dịch

Cài XeLaTeX, Biber, latexmk và các font Noto Serif, Noto Sans, DejaVu Sans Mono.

```bash
cd reports/terrain_ugv_2026
latexmk -xelatex main.tex
```

Hoặc sử dụng `xelatex main.tex`, `biber main`, `xelatex main.tex`, `xelatex main.tex`.

Trên Overleaf, chọn XeLaTeX và tài liệu gốc `main.tex`.

## Trước khi nộp

Điền thông tin tác giả/người hướng dẫn, kiểm tra lại metadata và đọc bài gốc từ DOI. Các câu hỏi trong phần research gap chỉ là giả thuyết cần đối chiếu và kiểm chứng bằng thực nghiệm. Chưa tuyên bố thành tích công bố Q1.

Cập nhật: 2026-10-08.
