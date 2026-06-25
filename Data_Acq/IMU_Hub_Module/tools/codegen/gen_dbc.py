#!/usr/bin/env python3
"""
DBC 파일 자동 생성 스크립트 (AGR-DOP V2 호환)
CAN Database (DBC) 파일을 IMU Hub Module의 AGR-DOP V2 구조에 맞게 생성합니다.
Vector CANalyzer, PCAN View, Kvaser Database Editor 등에서 사용 가능합니다.

[TPDO 구조] (Week 2 최신)
- TPDO Payload (64 bytes):
  - Metadata (4 bytes): timestamp_ms (24-bit) + valid_mask (8-bit)
  - IMU Data (60 bytes): 3개 IMU × 20 bytes
    - Quaternion (8 bytes): q[w,x,y,z] (int16 × 4)
    - Accel (6 bytes): a[x,y,z] (int16 × 3, 0.001g 단위)
    - Gyro (6 bytes): g[x,y,z] (int16 × 3, 0.01 deg/s 단위)

Usage:
    python gen_dbc.py <driver_file> <output_file>
    
Example:
    python gen_dbc.py ../IMU_Hub_FW/Devices/AGR/eXtension_Module/xm_drv.c output/IMU_Hub.dbc
"""

import re
import sys
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple
from datetime import datetime

@dataclass
class ODEntry:
    """Object Dictionary Entry"""
    index: int
    subindex: int
    name: str
    data_type: str
    size: int
    access: str
    
    def __repr__(self):
        return f"OD[0x{self.index:04X}:{self.subindex:02X}] {self.name}"


class DBCGenerator:
    """DBC 파일 생성기"""
    
    # Data Type → DBC Signal Type 매핑
    TYPE_MAP = {
        'AGR_TYPE_UINT8':   ('unsigned', 8, 1, 0),
        'AGR_TYPE_UINT16':  ('unsigned', 16, 1, 0),
        'AGR_TYPE_UINT32':  ('unsigned', 32, 1, 0),
        'AGR_TYPE_INT8':    ('signed', 8, 1, 0),
        'AGR_TYPE_INT16':   ('signed', 16, 1, 0),
        'AGR_TYPE_INT32':   ('signed', 32, 1, 0),
        'AGR_TYPE_FLOAT32': ('float', 32, 1, 0),
        'AGR_TYPE_BLOB':    ('unsigned', 64, 1, 0),  # 8 bytes binary
    }
    
    def __init__(self, module_name: str, node_id: int):
        self.module_name = module_name
        self.node_id = node_id
        self.entries: List[ODEntry] = []
        
    def parse_od_entries(self, file_path: str) -> int:
        """C 소스 파일에서 OD Entry 배열을 파싱"""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        pattern = r'\{\s*0x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)\s*,\s*(AGR_TYPE_\w+)\s*,\s*(\d+)\s*,\s*(AGR_ACCESS_\w+)\s*,\s*&([\w.\[\]]+)\s*,\s*(\w+|NULL)\s*\}'
        
        matches = re.finditer(pattern, content)
        
        for match in matches:
            index = int(match.group(1), 16)
            subindex = int(match.group(2), 16)
            data_type = match.group(3)
            size = int(match.group(4))
            access = match.group(5)
            var_name = match.group(6)
            
            name = self._clean_var_name(var_name)
            
            entry = ODEntry(
                index=index,
                subindex=subindex,
                name=name,
                data_type=data_type,
                size=size,
                access=access
            )
            
            self.entries.append(entry)
            
        return len(self.entries)
    
    def _clean_var_name(self, var_name: str) -> str:
        """변수명을 DBC Signal 이름으로 변환"""
        # 배열 인덱스 처리
        var_name = re.sub(r'\[(\d+)\]', r'_\1', var_name)
        
        # Prefix 제거
        var_name = re.sub(r'^s_(tx_data|rx_data)\.', '', var_name)
        var_name = re.sub(r'^s_', '', var_name)
        
        # snake_case를 CamelCase로 변환
        parts = var_name.split('.')
        name = '_'.join(part.title().replace('_', '') for part in parts)
        
        return name
    
    def generate_dbc(self, output_file: str):
        """DBC 파일 생성"""
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(self._generate_header())
            f.write(self._generate_nodes())
            f.write(self._generate_messages())
            f.write(self._generate_attributes())
            
        print(f"✅ DBC 파일 생성 완료: {output_file}")
        print(f"   - Module: {self.module_name}")
        print(f"   - Node ID: 0x{self.node_id:02X}")
        print(f"   - Entries: {len(self.entries)}")
        
    def _generate_header(self) -> str:
        """DBC 헤더 생성"""
        now = datetime.now().strftime("%a %b %d %H:%M:%S %Y")
        return f"""VERSION ""

NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_SGTYPE_REL_
    SG_MUL_VAL_

BS_:

BU_: {self.module_name} XM10 CM

"""
    
    def _generate_nodes(self) -> str:
        """노드 정보 생성"""
        return ""  # BU_ 라인에 이미 포함됨
    
    def _generate_messages(self) -> str:
        """CAN 메시지 및 시그널 생성"""
        result = ""
        
        # TPDO1 (IMU Group A: 0, 1, 2)
        result += self._generate_tpdo1()
        
        # TPDO2 (IMU Group B: 3, 4, 5)
        result += self._generate_tpdo2()
        
        # SDO Messages
        result += self._generate_sdo_messages()
        
        # Heartbeat
        result += self._generate_heartbeat()
        
        # Message Comments
        result += self._generate_message_comments()
        
        return result
    
    def _generate_tpdo1(self) -> str:
        """TPDO1 메시지 생성 (Group A: IMU 0, 1, 2)"""
        can_id = 0x180 + self.node_id
        result = f"BO_ {can_id} TPDO1_ImuData_GroupA: 64 {self.module_name}\n"
        
        # Metadata (4 bytes)
        result += f" SG_ Timestamp_ms : 0|24@1+ (1,0) [0|16777215] \"ms\" XM10\n"
        result += f" SG_ ValidMask : 24|8@1+ (1,0) [0|255] \"\" XM10\n"
        
        # IMU 데이터 (각 20 bytes: q[4] + a[3] + g[3])
        bit_pos = 32  # 메타데이터 이후
        for i in range(3):  # IMU 0, 1, 2
            # Quaternion (int16 × 4 = 8 bytes)
            result += f" SG_ IMU{i}_Quat_W : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_Quat_X : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_Quat_Y : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_Quat_Z : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            
            # Acceleration (int16 × 3 = 6 bytes, 0.001g 단위)
            result += f" SG_ IMU{i}_AccX : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"g\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_AccY : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"g\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_AccZ : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"g\" XM10\n"
            bit_pos += 16
            
            # Gyroscope (int16 × 3 = 6 bytes, 0.01 deg/s 단위)
            result += f" SG_ IMU{i}_GyroX : {bit_pos}|16@1- (0.01,0) [-327.68|327.67] \"deg/s\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_GyroY : {bit_pos}|16@1- (0.01,0) [-327.68|327.67] \"deg/s\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_GyroZ : {bit_pos}|16@1- (0.01,0) [-327.68|327.67] \"deg/s\" XM10\n"
            bit_pos += 16
        
        result += "\n"
        return result
    
    def _generate_tpdo2(self) -> str:
        """TPDO2 메시지 생성 (Group B: IMU 3, 4, 5)"""
        can_id = 0x280 + self.node_id
        result = f"BO_ {can_id} TPDO2_ImuData_GroupB: 64 {self.module_name}\n"
        
        # Metadata (4 bytes)
        result += f" SG_ Timestamp_ms : 0|24@1+ (1,0) [0|16777215] \"ms\" XM10\n"
        result += f" SG_ ValidMask : 24|8@1+ (1,0) [0|255] \"\" XM10\n"
        
        # IMU 데이터 (각 20 bytes: q[4] + a[3] + g[3])
        bit_pos = 32  # 메타데이터 이후
        for i in range(3, 6):  # IMU 3, 4, 5
            # Quaternion (int16 × 4 = 8 bytes)
            result += f" SG_ IMU{i}_Quat_W : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_Quat_X : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_Quat_Y : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_Quat_Z : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"\" XM10\n"
            bit_pos += 16
            
            # Acceleration (int16 × 3 = 6 bytes, 0.001g 단위)
            result += f" SG_ IMU{i}_AccX : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"g\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_AccY : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"g\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_AccZ : {bit_pos}|16@1- (0.001,0) [-32.768|32.767] \"g\" XM10\n"
            bit_pos += 16
            
            # Gyroscope (int16 × 3 = 6 bytes, 0.01 deg/s 단위)
            result += f" SG_ IMU{i}_GyroX : {bit_pos}|16@1- (0.01,0) [-327.68|327.67] \"deg/s\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_GyroY : {bit_pos}|16@1- (0.01,0) [-327.68|327.67] \"deg/s\" XM10\n"
            bit_pos += 16
            result += f" SG_ IMU{i}_GyroZ : {bit_pos}|16@1- (0.01,0) [-327.68|327.67] \"deg/s\" XM10\n"
            bit_pos += 16
        
        result += "\n"
        return result
    
    def _generate_sdo_messages(self) -> str:
        """SDO 메시지 생성"""
        sdo_rx_id = 0x600 + self.node_id
        sdo_tx_id = 0x580 + self.node_id
        
        result = f"""BO_ {sdo_rx_id} SDO_RX: 8 XM10
 SG_ SDO_CS : 0|8@1+ (1,0) [0|255] "" {self.module_name}
 SG_ SDO_Index : 8|16@1+ (1,0) [0|65535] "" {self.module_name}
 SG_ SDO_Subindex : 24|8@1+ (1,0) [0|255] "" {self.module_name}
 SG_ SDO_Data : 32|32@1+ (1,0) [0|4294967295] "" {self.module_name}

BO_ {sdo_tx_id} SDO_TX: 8 {self.module_name}
 SG_ SDO_CS : 0|8@1+ (1,0) [0|255] "" XM10
 SG_ SDO_Index : 8|16@1+ (1,0) [0|65535] "" XM10
 SG_ SDO_Subindex : 24|8@1+ (1,0) [0|255] "" XM10
 SG_ SDO_Data : 32|32@1+ (1,0) [0|4294967295] "" XM10

"""
        return result
    
    def _generate_heartbeat(self) -> str:
        """Heartbeat 메시지 생성"""
        hb_id = 0x700 + self.node_id
        
        result = f"""BO_ {hb_id} Heartbeat: 1 {self.module_name}
 SG_ NMT_State : 0|8@1+ (1,0) [0|127] "" XM10

"""
        return result
    
    def _generate_message_comments(self) -> str:
        """메시지 설명 추가"""
        hb_id = 0x700 + self.node_id
        
        result = f"""CM_ SG_ {hb_id} NMT_State "0=Boot-up, 4=Stopped, 5=Operational, 127=Pre-Operational";

"""
        return result
    
    def _generate_attributes(self) -> str:
        """DBC 속성 정의"""
        return """BA_DEF_ "BusType" STRING;
BA_ "BusType" "CAN FD";

BA_DEF_ BO_ "GenMsgCycleTime" INT 0 100000;
BA_DEF_ BO_ "GenMsgSendType" STRING;

BA_ "GenMsgCycleTime" BO_ {tpdo1_id} 1;
BA_ "GenMsgSendType" BO_ {tpdo1_id} "cyclic";
BA_ "GenMsgCycleTime" BO_ {tpdo2_id} 1;
BA_ "GenMsgSendType" BO_ {tpdo2_id} "cyclic";
BA_ "GenMsgCycleTime" BO_ {hb_id} 100;
BA_ "GenMsgSendType" BO_ {hb_id} "cyclic";

BA_DEF_DEF_ "BusType" "";
BA_DEF_DEF_ "GenMsgCycleTime" 0;
BA_DEF_DEF_ "GenMsgSendType" "";
""".format(
            tpdo1_id=0x180 + self.node_id,
            tpdo2_id=0x280 + self.node_id,
            hb_id=0x700 + self.node_id
        )


def main():
    if len(sys.argv) < 3:
        print("Usage: python gen_dbc.py <driver_file> <output_file>")
        print("Example: python gen_dbc.py ../IMU_Hub_FW/Devices/AGR/eXtension_Module/xm_drv.c output/IMU_Hub.dbc")
        sys.exit(1)
    
    driver_file = sys.argv[1]
    output_file = sys.argv[2]
    
    if not os.path.exists(driver_file):
        print(f"❌ 파일을 찾을 수 없습니다: {driver_file}")
        sys.exit(1)
    
    # 출력 디렉토리 생성
    os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else '.', exist_ok=True)
    
    # DBC 생성
    generator = DBCGenerator(
        module_name="IMU_Hub",
        node_id=0x08  # AGR_NODE_ID_IMU_HUB
    )
    
    # OD Entry 파싱
    count = generator.parse_od_entries(driver_file)
    print(f"📊 파싱 완료: {count}개의 OD Entry 발견")
    
    # DBC 생성
    generator.generate_dbc(output_file)
    
    print("\n✅ DBC 파일 생성 완료!")
    print(f"   Vector CANalyzer, PCAN View 또는 Kvaser Database Editor에서 '{output_file}'를 열어보세요.")


if __name__ == '__main__':
    main()

