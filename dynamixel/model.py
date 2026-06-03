"""Dynamixel model number constants.

Each class attribute maps a human-readable model name to its protocol
model number (uint16 returned by the PING instruction).  Used by
``get_control_table()`` to select the correct register layout.
"""


class DynamixelModel:
    """Lookup table of known Dynamixel model numbers."""

    UNKNOWN = 0

    # ── AX / DX / RX series (Protocol 1.0) ──────────────────────────
    AX12A = 12
    AX12W = 300
    AX18A = 18
    DX113 = 113
    DX116 = 116
    DX117 = 117
    RX10 = 10
    RX24F = 24
    RX28 = 28
    RX64 = 64
    EX106 = 107

    # ── MX series (Protocol 1.0 / 1.1) ──────────────────────────────
    MX12W = 360
    MX28 = 29
    MX64 = 310
    MX106 = 320

    # ── MX series (Protocol 2.0 firmware) ────────────────────────────
    MX28_2 = 30
    MX64_2 = 311
    MX106_2 = 321

    # ── XL / XC 320–430 series ───────────────────────────────────────
    XL320 = 350
    XC430_W150 = 1070
    XC430_W240 = 1080
    XL430_W250 = 1060
    XXL430_W250 = 1090
    XXC430_W250 = 1160

    # ── XL / XC 330 series ───────────────────────────────────────────
    XL330_M077 = 1190
    XL330_M288 = 1200
    XC330_T181 = 1210
    XC330_T288 = 1220
    XC330_M181 = 1230
    XC330_M288 = 1240

    # ── XM / XH / XD 430 series ─────────────────────────────────────
    XM430_W210 = 1030
    XM430_W350 = 1020
    XH430_V210 = 1050
    XH430_V350 = 1040
    XH430_W210 = 1010
    XH430_W350 = 1000
    XD430_T210 = 1011
    XD430_T350 = 1001

    # ── XM / XH / XD 540 series ─────────────────────────────────────
    XM540_W150 = 1130
    XM540_W270 = 1120
    XH540_W150 = 1110
    XH540_W270 = 1100
    XH540_V150 = 1150
    XH540_V270 = 1140
    XD540_T150 = 1111
    XD540_T270 = 1101

    # ── XW series ────────────────────────────────────────────────────
    XW540_T140 = 1180
    XW540_T260 = 1170
    XW430_T200 = 1280
    XW430_T333 = 1270

    # ── PRO series (R firmware) ──────────────────────────────────────
    PRO_M42_10_S260_R = 43288
    PRO_M54_40_S250_R = 46096
    PRO_M54_60_S250_R = 46352
    PRO_H42_20_S300_R = 51200
    PRO_H54_100_S500_R = 53768
    PRO_H54_200_S500_R = 54024

    # ── PRO series (RA firmware) / PRO+ ──────────────────────────────
    PRO_M42_10_S260_RA = 43289
    PRO_M54_40_S250_RA = 46097
    PRO_M54_60_S250_RA = 46353
    PRO_H42_20_S300_RA = 51201
    PRO_H54_100_S500_RA = 53761
    PRO_H54_200_S500_RA = 54025
    PRO_H42P_020_S300_R = 2000
    PRO_H54P_100_S500_R = 2010
    PRO_H54P_200_S500_R = 2020
    PRO_M42P_010_S260_R = 2100
    PRO_M54P_040_S250_R = 2110
    PRO_M54P_060_S250_R = 2120

    # ── Y series ─────────────────────────────────────────────────────
    YM070_210_M001_RH = 4000
    YM070_210_B001_RH = 4010
    YM070_210_R051_RH = 4020
    YM070_210_R099_RH = 4030
    YM070_210_A051_RH = 4040
    YM070_210_A099_RH = 4050
    YM080_230_M001_RH = 4120
    YM080_230_B001_RH = 4130
    YM080_230_R051_RH = 4140
    YM080_230_R099_RH = 4150
    YM080_230_A051_RH = 4160
    YM080_230_A099_RH = 4170

    @classmethod
    def get_name(cls, model_number):
        """Get the model name for a given model number.
        
        Args:
            model_number: The model number (integer).
            
        Returns:
            The model name as a string, or "UNKNOWN" if not found.
        """
        for name, number in cls.__dict__.items():
            if isinstance(number, int) and number == model_number:
                return name
        return "UNKNOWN"
