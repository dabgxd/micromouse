// Kiểm tra module Bluetooth BLE trên xe (bản clone "HC-05" chạy BLE, tên MICROMOUSE) từ macOS.
//   swift tools/hwtest_web/ble_check.swift scan            -> quét BLE 15 s, tìm module serial BLE
//   swift tools/hwtest_web/ble_check.swift gatt [tên]      -> kết nối, liệt kê service/characteristic, nghe dữ liệu, thử ping
// Web test kết nối trực tiếp bằng nút "Kết nối BLE" (kênh FFE0 / FFE1); script này chỉ để chẩn đoán.
import CoreBluetooth
import Foundation

class BleScan: NSObject, CBCentralManagerDelegate {
    var seen = [UUID: (String, Int, [String])]()
    var cm: CBCentralManager!
    override init() { super.init(); cm = CBCentralManager(delegate: self, queue: nil) }
    func centralManagerDidUpdateState(_ c: CBCentralManager) {
        guard c.state == .poweredOn else { print("Bluetooth chưa bật hoặc chưa được cấp quyền (state \(c.state.rawValue))"); exit(1) }
        print("Đang quét BLE 15 s...")
        c.scanForPeripherals(withServices: nil, options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
        DispatchQueue.main.asyncAfter(deadline: .now() + 15) { self.finish() }
    }
    func centralManager(_ c: CBCentralManager, didDiscover p: CBPeripheral, advertisementData ad: [String: Any], rssi: NSNumber) {
        let name = p.name ?? (ad[CBAdvertisementDataLocalNameKey] as? String) ?? "(không tên)"
        let svcs = (ad[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID] ?? []).map { $0.uuidString }
        if seen[p.identifier] == nil { print("  \(name)  RSSI \(rssi)  dịch vụ \(svcs)") }
        seen[p.identifier] = (name, rssi.intValue, svcs)
    }
    func finish() {
        let hits = seen.values.filter { n, _, s in
            n.range(of: "HC|MICRO|BT0|JDY|MLT|HM-|AT-09", options: .regularExpression) != nil || s.contains("FFE0")
        }
        print("Quét xong: \(seen.count) thiết bị BLE.")
        print(hits.isEmpty ? "Không thấy module serial BLE quen thuộc (HC-05 BLE / HM-10 / JDY / FFE0)."
                           : "Có vẻ là module serial BLE: \(hits.map { $0.0 })")
        exit(0)
    }
}

/// Kết nối module BLE theo tên, liệt kê service/characteristic, nghe dữ liệu 4 s và thử gửi "ping".
class BleGatt: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    let name: String
    var cm: CBCentralManager!
    var per: CBPeripheral?
    var pending = 0
    var writable: [CBCharacteristic] = []
    init(name: String) { self.name = name; super.init(); cm = CBCentralManager(delegate: self, queue: nil) }
    func centralManagerDidUpdateState(_ c: CBCentralManager) {
        guard c.state == .poweredOn else { print("Bluetooth chưa bật / chưa có quyền"); exit(1) }
        print("Đang tìm \(name)...")
        c.scanForPeripherals(withServices: nil)
        DispatchQueue.main.asyncAfter(deadline: .now() + 15) { if self.per == nil { print("Không thấy \(self.name)"); exit(1) } }
    }
    func centralManager(_ c: CBCentralManager, didDiscover p: CBPeripheral, advertisementData ad: [String: Any], rssi: NSNumber) {
        let n = p.name ?? (ad[CBAdvertisementDataLocalNameKey] as? String) ?? ""
        guard n == name, per == nil else { return }
        per = p; c.stopScan(); p.delegate = self
        print("Thấy \(n) (RSSI \(rssi)), đang kết nối...")
        c.connect(p)
    }
    func centralManager(_ c: CBCentralManager, didConnect p: CBPeripheral) { print("Đã kết nối."); p.discoverServices(nil) }
    func centralManager(_ c: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) { print("Kết nối lỗi: \(String(describing: error))"); exit(1) }
    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        let s = p.services ?? []; pending = s.count
        for sv in s { p.discoverCharacteristics(nil, for: sv) }
    }
    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor sv: CBService, error: Error?) {
        print("Service \(sv.uuid.uuidString)")
        for ch in sv.characteristics ?? [] {
            var props: [String] = []
            let pr = ch.properties
            if pr.contains(.read) { props.append("read") }
            if pr.contains(.write) { props.append("write") }
            if pr.contains(.writeWithoutResponse) { props.append("writeNoResp") }
            if pr.contains(.notify) { props.append("notify") }
            if pr.contains(.indicate) { props.append("indicate") }
            print("   characteristic \(ch.uuid.uuidString)  [\(props.joined(separator: ", "))]")
            if pr.contains(.notify) || pr.contains(.indicate) { p.setNotifyValue(true, for: ch) }
            if pr.contains(.write) || pr.contains(.writeWithoutResponse) { writable.append(ch) }
        }
        pending -= 1
        if pending == 0 {
            print("Nghe dữ liệu 8 s (firmware hwtest gửi ENC 25 lần/giây), giây thứ 4 gửi \"ping\"...")
            DispatchQueue.main.asyncAfter(deadline: .now() + 4) {
                for ch in self.writable {
                    let t: CBCharacteristicWriteType = ch.properties.contains(.writeWithoutResponse) ? .withoutResponse : .withResponse
                    p.writeValue("ping\n".data(using: .utf8)!, for: ch, type: t)
                    print("-> gửi ping qua \(ch.uuid.uuidString)")
                }
            }
            DispatchQueue.main.asyncAfter(deadline: .now() + 8) {
                print("Xong. Tổng nhận \(self.total) byte qua BLE.")
                if self.total == 0 { print("=> Không có byte nào từ xe: kiểm tra firmware hwtest đang chạy (LED nháy), dây TXD/RXD, nguồn module.") }
                exit(0)
            }
        }
    }
    var shown = 0, total = 0
    func peripheral(_ p: CBPeripheral, didUpdateNotificationStateFor ch: CBCharacteristic, error: Error?) {
        print(error == nil ? "   (đã bật nhận dữ liệu từ \(ch.uuid.uuidString))" : "   bật notify \(ch.uuid.uuidString) lỗi: \(error!)")
    }
    func peripheral(_ p: CBPeripheral, didUpdateValueFor ch: CBCharacteristic, error: Error?) {
        guard let d = ch.value else { return }
        total += d.count
        let printable = d.allSatisfy { $0 == 10 || $0 == 13 || (32...126).contains($0) }
        let isEnc = printable && String(decoding: d, as: UTF8.self).hasPrefix("ENC,")
        if isEnc { guard shown < 5 else { return }; shown += 1 }  // ENC lặp 25 lần/s: chỉ in 5 dòng đầu, dòng khác (PONG...) luôn in
        let s = printable ? String(decoding: d, as: UTF8.self).replacingOccurrences(of: "\n", with: "⏎")
                          : "HEX " + d.map { String(format: "%02X", $0) }.joined(separator: " ") + "  (ký tự rác: lệch baud?)"
        print("<- \(ch.uuid.uuidString): \(s)")
    }
}

let args = CommandLine.arguments
if args.count >= 2 && args[1] == "gatt" {
    let g = BleGatt(name: args.count >= 3 ? args[2] : "MICROMOUSE")
    _ = g
    RunLoop.main.run()
} else if args.count >= 2 && args[1] == "scan" {
    let b = BleScan()
    _ = b
    RunLoop.main.run()
} else {
    print("Dùng: swift ble_check.swift scan | gatt [tên]")
}
