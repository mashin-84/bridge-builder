/*
 * 蓋橋遊戲 全站共用設定
 * 更換 Google Apps Script 計數網址時，只需修改這裡的 ANALYTICS_URL。
 * 修改後請把各頁面中 config.js?v=1 的版本號加 1，避免瀏覽器使用舊快取。
 */
window.BridgeConfig = {
  ANALYTICS_URL: "https://script.google.com/macros/s/AKfycbxaM_d8WTJHj-B8FSNiwODChDVw5C912tL4zJIxiVyexdSMeG7kDaUwuS1-u52d6hiiww/exec",

  // 送出一次計數；任何失敗都靜默忽略，不影響遊戲運作
  trackHit(type) {
    try {
      fetch(this.ANALYTICS_URL + "?action=hit&type=" + encodeURIComponent(type), { mode: "no-cors" })
        .catch(() => {});
    } catch (err) {}
  }
};
