#!/bin/sh
# Mở bảng điều khiển test: bật server web (nếu chưa chạy) rồi mở Chrome.
# Dùng: tools/hwtest_web/serve.sh
cd "$(dirname "$0")" || exit 1
PORT=8000
URL="http://localhost:$PORT"

open_browser() { open -a "Google Chrome" "$URL" 2>/dev/null || open "$URL"; }

if lsof -nP -iTCP:$PORT -sTCP:LISTEN >/dev/null 2>&1; then
  echo "Server đã chạy sẵn ở $URL, chỉ mở trình duyệt."
  open_browser
else
  echo "Đang mở $URL (bấm Ctrl+C để tắt server)."
  (sleep 1 && open_browser) &
  exec python3 -m http.server $PORT
fi
