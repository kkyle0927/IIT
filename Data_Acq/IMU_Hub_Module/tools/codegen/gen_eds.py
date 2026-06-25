#!/usr/bin/env python3
"""
EDS 파일 자동 생성 스크립트 (AGR-DOP V2 호환)
CANopen Electronic Data Sheet (EDS) 파일을 IMU Hub Module의 OD Entry로부터 자동 생성합니다.

[AGR-DOP V2 구조]
- Node ID: 0x08 (IMU Hub)
- TPDO1 (0x188): Group A (IMU 0, 1, 2) - 64 bytes
- TPDO2 (0x288): Group B (IMU 3, 4, 5) - 64 bytes
- SDO (0x608/0x588): Object Dictionary 접근
- Heartbeat (0x708): 연결 상태

[TPDO Payload 구조] (Week 2)
- Metadata (4 bytes): timestamp_ms (24-bit) + valid_mask (8-bit)
- IMU Data (60 bytes): 3개 IMU × 20 bytes
  - Quaternion (8 bytes): q[w,x,y,z] (int16 × 4)
  - Accel (6 bytes): a[x,y,z] (int16 × 3, 0.001g 단위)
  - Gyro (6 bytes): g[x,y,z] (int16 × 3, 0.01 deg/s 단위)

Usage:
    python gen_eds.py <driver_file> <output_file>
    
Example:
    python gen_eds.py ../IMU_Hub_FW/Devices/AGR/eXtension_Module/xm_drv.c output/IMU_Hub.eds
"""

import re
import sys
import os
from dataclasses import dataclass
from typing import List, Optional
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
    default_value: str = "0"
    pdo_mappable: bool = True
    
    def __repr__(self):
        return f"OD[0x{self.index:04X}:{self.subindex:02X}] {self.name} ({self.data_type})"


class EDSGenerator:
    """EDS 파일 생성기"""
    
    # CANopen Data Type 매핑
    TYPE_MAP = {
        'AGR_TYPE_UINT8':   ('UNSIGNED8',  '0x0005', 1),
        'AGR_TYPE_UINT16':  ('UNSIGNED16', '0x0006', 2),
        'AGR_TYPE_UINT32':  ('UNSIGNED32', '0x0007', 4),
        'AGR_TYPE_INT8':    ('INTEGER8',   '0x0002', 1),
        'AGR_TYPE_INT16':   ('INTEGER16',  '0x0003', 2),
        'AGR_TYPE_INT32':   ('INTEGER32',  '0x0004', 4),
        'AGR_TYPE_FLOAT32': ('REAL32',     '0x0008', 4),
        'AGR_TYPE_BLOB':    ('DOMAIN',     '0x000F', 0),  # Variable size
    }
    
    ACCESS_MAP = {
        'AGR_ACCESS_RO': 'ro',
        'AGR_ACCESS_WO': 'wo',
        'AGR_ACCESS_RW': 'rw',
    }
    
    def __init__(self, module_name: str, node_id: int, vendor_name: str = "Angel Robotics Co., Ltd."):
        self.module_name = module_name
        self.node_id = node_id
        self.vendor_name = vendor_name
        self.entries: List[ODEntry] = []
        
    def parse_od_entries(self, file_path: str) -> int:
        """
        C 소스 파일에서 OD Entry 배열을 파싱합니다.
        
        패턴 예:
        { 0x2000, 0x00, AGR_TYPE_UINT16, 2, AGR_ACCESS_RW, &s_rx_data.sample_rate, _OnSampleRateChanged },
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # OD Entry 패턴 검색
        pattern = r'\{\s*0x([0-9A-Fa-f]+)\s*,\s*0x([0-9A-Fa-f]+)\s*,\s*(AGR_TYPE_\w+)\s*,\s*(\d+)\s*,\s*(AGR_ACCESS_\w+)\s*,\s*&([\w.\[\]]+)\s*,\s*(\w+|NULL)\s*\}'
        
        matches = re.finditer(pattern, content)
        
        for match in matches:
            index = int(match.group(1), 16)
            subindex = int(match.group(2), 16)
            data_type = match.group(3)
            size = int(match.group(4))
            access = match.group(5)
            var_name = match.group(6)
            
            # 변수명에서 이름 추출
            name = self._clean_var_name(var_name)
            
            entry = ODEntry(
                index=index,
                subindex=subindex,
                name=name,
                data_type=data_type,
                size=size,
                access=access,
                default_value='0',
                pdo_mappable=True
            )
            
            self.entries.append(entry)
            
        return len(self.entries)
    
    def _clean_var_name(self, var_name: str) -> str:
        """변수명을 사람이 읽기 쉬운 이름으로 변환"""
        # s_tx_data.imu[0] -> IMU_0_Data
        # s_rx_data.sample_rate -> Sample_Rate
        
        # 배열 인덱스 처리
        var_name = re.sub(r'\[(\d+)\]', r'_\1', var_name)
        
        # Prefix 제거 (s_, s_tx_data., s_rx_data., etc.)
        var_name = re.sub(r'^s_(tx_data|rx_data)\.', '', var_name)
        var_name = re.sub(r'^s_', '', var_name)
        
        # snake_case를 Pascal_Case로 변환
        parts = var_name.split('.')
        name = '_'.join(part.title().replace('_', '') for part in parts)
        
        return name
    
    def generate_eds(self, output_file: str):
        """EDS 파일 생성"""
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(self._generate_device_info())
            f.write(self._generate_dummy_usage())
            f.write(self._generate_comments())
            f.write(self._generate_mandatory_objects())
            f.write(self._generate_manufacturer_objects())
            
        print(f"✅ EDS 파일 생성 완료: {output_file}")
        print(f"   - Module: {self.module_name}")
        print(f"   - Node ID: 0x{self.node_id:02X}")
        print(f"   - Entries: {len(self.entries)}")
        
    def _generate_device_info(self) -> str:
        """[DeviceInfo] 섹션 생성"""
        return f"""[DeviceInfo]
VendorName={self.vendor_name}
ProductName={self.module_name}
OrderCode=
VendorNumber=0x00000ACE
ProductNumber=0x{self.node_id:08X}
RevisionNumber=0x00020000
SerialNumber=
BaudRate_10=1
BaudRate_20=1
BaudRate_50=1
BaudRate_125=1
BaudRate_250=1
BaudRate_500=1
BaudRate_800=1
BaudRate_1000=1
SimpleBootUpMaster=0
SimpleBootUpSlave=1
Granularity=8
DynamicChannelsSupported=0
CompactPDO=0
GroupMessaging=0
Features=0
LSS_Supported=0
NrOfRXPDO=4
NrOfTXPDO=4

"""
    
    def _generate_dummy_usage(self) -> str:
        """[DummyUsage] 섹션 생성"""
        return """[DummyUsage]
Dummy0001=0
Dummy0002=0
Dummy0003=0
Dummy0004=0
Dummy0005=1
Dummy0006=1
Dummy0007=1

"""
    
    def _generate_comments(self) -> str:
        """[Comments] 섹션 생성"""
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return f"""[Comments]
Lines=1
Line1=Generated by Angel Robotics EDS Generator at {now}

"""
    
    def _generate_mandatory_objects(self) -> str:
        """[MandatoryObjects] 섹션 생성"""
        result = """[MandatoryObjects]
SupportedObjects=3
1=0x1000
2=0x1001
3=0x1018

[1000]
ParameterName=Device Type
ObjectType=0x07
DataType=0x0007
AccessType=ro
DefaultValue=0x00000000
PDOMapping=0

[1001]
ParameterName=Error Register
ObjectType=0x07
DataType=0x0005
AccessType=ro
DefaultValue=0
PDOMapping=1

[1018]
ParameterName=Identity Object
ObjectType=0x09
SubNumber=5

[1018sub0]
ParameterName=Number of entries
ObjectType=0x07
DataType=0x0005
AccessType=ro
DefaultValue=4
PDOMapping=0

[1018sub1]
ParameterName=Vendor ID
ObjectType=0x07
DataType=0x0007
AccessType=ro
DefaultValue=0x00000ACE
PDOMapping=0

[1018sub2]
ParameterName=Product Code
ObjectType=0x07
DataType=0x0007
AccessType=ro
DefaultValue=0x{node_id:08X}
PDOMapping=0

[1018sub3]
ParameterName=Revision number
ObjectType=0x07
DataType=0x0007
AccessType=ro
DefaultValue=0x00020000
PDOMapping=0

[1018sub4]
ParameterName=Serial number
ObjectType=0x07
DataType=0x0007
AccessType=ro
DefaultValue=0x00000001
PDOMapping=0

""".format(node_id=self.node_id)
        return result
    
    def _generate_manufacturer_objects(self) -> str:
        """[ManufacturerObjects] 섹션 생성 (실제 OD Entries)"""
        result = f"[ManufacturerObjects]\nSupportedObjects={len(self.entries)}\n"
        
        for i, entry in enumerate(self.entries, start=1):
            result += f"{i}=0x{entry.index:04X}\n"
        
        result += "\n"
        
        # 각 OD Entry 상세 정보
        for entry in self.entries:
            result += self._generate_od_entry(entry)
        
        return result
    
    def _generate_od_entry(self, entry: ODEntry) -> str:
        """개별 OD Entry 정보 생성"""
        type_name, type_code, default_size = self.TYPE_MAP.get(entry.data_type, ('UNSIGNED32', '0x0007', 4))
        access_type = self.ACCESS_MAP.get(entry.access, 'rw')
        
        # BLOB (Domain) 타입은 크기를 사용자 정의대로
        if entry.data_type == 'AGR_TYPE_BLOB':
            default_size = entry.size
        
        return f"""[{entry.index:04X}]
ParameterName={entry.name}
ObjectType=0x07
DataType={type_code}
AccessType={access_type}
DefaultValue={entry.default_value}
PDOMapping={1 if entry.pdo_mappable else 0}

"""


def main():
    if len(sys.argv) < 3:
        print("Usage: python gen_eds.py <driver_file> <output_file>")
        print("Example: python gen_eds.py ../IMU_Hub_FW/Devices/AGR/eXtension_Module/xm_drv.c output/IMU_Hub.eds")
        sys.exit(1)
    
    driver_file = sys.argv[1]
    output_file = sys.argv[2]
    
    if not os.path.exists(driver_file):
        print(f"❌ 파일을 찾을 수 없습니다: {driver_file}")
        sys.exit(1)
    
    # 출력 디렉토리 생성
    os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else '.', exist_ok=True)
    
    # EDS 생성
    generator = EDSGenerator(
        module_name="IMU_Hub_Module",
        node_id=0x08,  # AGR_NODE_ID_IMU_HUB
        vendor_name="Angel Robotics Co., Ltd."
    )
    
    # OD Entry 파싱
    count = generator.parse_od_entries(driver_file)
    print(f"📊 파싱 완료: {count}개의 OD Entry 발견")
    
    # EDS 생성
    generator.generate_eds(output_file)
    
    print("\n✅ EDS 파일 생성 완료!")
    print(f"   Vector CANdb++ 또는 CANopen 도구에서 '{output_file}'를 열어보세요.")


if __name__ == '__main__':
    main()

