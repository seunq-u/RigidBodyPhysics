# config.py
# 각종 물리 상수 정의
# 섭씨 20도 및 표준압(1 atm = 101325 Pa) 기준 / 습도 무시

# 중력 가속도 [m/s^2]
g = 9.80665
gravitational_acceleration = g

# 회전 감쇠 계수(임의 값)
C_rot = 0.01

# 밀도 ρ [kg/m^3]
class Density:
    class Gas: 
            # 대기 구성 성분
            Air = 1.204 # 대기
            Nitrogen = 1.165 # 질소
            Oxygen = 1.331 # 산소
            Argon = 1.661 # 아르곤
            Carbon_Dioxide = 1.842 # 이산화탄소

            # 가벼운 기체
            Hydrogen = 0.084 # 수소
            Helium = 0.166 # 헬륨
            Neon = 0.838 # 네온

            # 연료 및 기타 기체
            Methane = 0.668 # 천연가스(메테인)
            Propane = 1.882 # 석유가스(프로판)
            Butane = 2.489 # 뷰테인
            Ammonia = 0.717 # 암모니아
            Chlorine = 2.994 # 염소
            Radon = 9.23 # 라돈
            Steam_100C = 0.597 # 수증기 (섭씨 100도 기준)

    class Liquid:
        class Water:
            Pure = 998.207
            Sea = 1025 # 바닷물 (1020~1030)
            Distilled = 997 # 증류수
            Heavy = 1105 # 중수 (D2O)

        class Fuel:
            Gasoline = 740 # 휘발유 (720~775)
            Diesel = 830 # 경유 (820~860)
            Kerosene = 810 # 등유
            Jet_Fuel = 800 # 항공유 (Jet A-1)
            LNG_Liquid = 447 # 액화천연가스 (액화메테인)
            LPG_Liquid = 510 # 액화석유가스 (액화프로판)

        class Alcohol:
            Ethanol = 789 # 에탄올
            Methanol = 792 # 메탄올
            Isopropyl = 786 # 이소프로필 알코올

        class Oil:
            Olive = 915 # 올리브유
            Vegetable = 920 # 식용유 (대두유)
            Engine_10W30 = 875 # 엔진오일 (870~890)
            Crude_Light = 850 # 경질 원유
            Crude_Heavy = 970 # 중질 원유

        class Household_Chemical: # 가정용 화학물질
            Glycerin = 1261 # 글리세린 
            Honey = 1420 # 꿀 (1400~1450)
            Milk = 1030 # 우유
            Vinegar = 1010 # 식초
            Mercury = 13546 # 수은
            Blood = 1060 # 혈액 (인간)

    class Solid:
        class Wood: # 섭씨 21도 65%습도의 12% 함수율 기준 https://blog.naver.com/robingoody/220939148564
            Zelkova_Serrata = 720 # 느티나무 (13% 함수율)
            Pinus_Densiflora = 460 # 소나무
            Quercus_Acutissima = 750 # 한국 참나무(참나무속 상수리나무)
            Quercus_Rubra = (red_oak := 650) # 루브라참나무(레드 오크)
            Quercus_Alba = (White_Oak := 640) # 흰참나무(화이트 오크)
            Cherry = 540 # 벚나무
            American_Walnut = 580 # 호두나무
            Birch = 610 # 자작나무
            Sweet_Chestnut = 540 # 밤나무
            Beech = 710 # 너도밤나무
            Hard_Maple = 640 # 단풍나무
            Persimmon = 780 # 감나무
            Cedar = 320 # 동양 삼나무
            Guibourtia_Tessmannii = 910 # 구이부르티아 테스마니(부빙가)
            Horse_Chestnut = 490 # 마로니에(가시칠엽수)
            Dalbergia_Latifolia = (Indian_Rosewood := 830) # 북인도 자단나무(북인도황단)

        class Material: # 금속
            class Iron: # 철류
                Pure = 7870 # 순철
                Wrought = 7800 # 연철 (7800~7870)
                Steel = 7850 # 강철(탄소강 기준)
                Cast = 7200 # 주철(무쇠 7100~7300)
                Pig = 7100 # 선철(무쇠 7000~7200)
            class Alloy: # 합금
                Stainless_Steel = 7930 # 스테인리스강 (SUS304 기준, 7700~8000)
                Brass = 8500 # 황동 (구리+아연, 8400~8700)
                Bronze = 8800 # 청동 (구리+주석, 8700~8900)
                Duralumin = 2800 # 두랄루민 (강력 알루미늄 합금)
                Inconel = 8250 # 인코넬 (니켈 합금)
            class Common: # 일반 금속
                Aluminum = 2700 # 알루미늄
                Copper = 8960 # 구리
                Lead = 11340 # 납
                Nickel = 8900 # 니켈
                Zinc = 7133 # 아연
                Tin = 7280 # 주석
                Titanium = 4506 # 티타늄 (강철보다 가볍고 강함)
                Magnesium = 1738 # 마그네슘 (실용 금속 중 가장 가벼움)
            class Precious_Metal: # 귀금속
                Gold = 19300 # 금
                Silver = 10490 # 은
                Platinum = 21450 # 백금
                Palladium = 12020 # 팔라듐
            class Heavy_Metal: # 중금속
                Tungsten = 19250 # 텅스텐 (매우 무거움)
                Mercury = 13546 # 수은 (상온 액체 상태)
                Uranium = 19050 # 우라늄
                Osmium = 22570 # 오스뮴 (지구상에서 가장 밀도가 높은 원소)

        class Rock: # 돌
            class Igneous: # 화성암
                Granite = 2700 # 화강암 (2600~2800)
                Andesite = 2695 # 안산암 (2250~3140)
                Basalt = 2750 # 현문암
            class Sedimentary: # 퇴적암
                Mudstone = 2600 # 이암
                Shale = 2600 # 셰일(혈암)
                Sandstone = 2500 # 사암 (2200~2800)
                Conglomerate = 2650 # 역암(2500~2800)
                Limestone = 2100 # 석회암(1500~2700, 석회암)
                Tuff = 2450 # 응회암 (1600~3300)
                Chert = 2800 # 각암
                Halite = 2168 # 암염
                Dolomite = 2950 # 백운암 (2900 ~ 3000, 칼슘 마그네슘 탄산염)
            class Metamorphic: # 변성암
                Marble = 2720 # 대리석
                Quartzite = 2665 # 규암
                Slate = 2700 # 점판암
                Schist = 2850 # 편암
                Gneiss = 2750 # 편마암
            Quartz = 2650 # 석영(이산화규소)

# 점성 계수(절대 점도) μ (섭씨 20도 기준) [Pa⋅s]
class Absolute_Viscosity:
    PureWater = 0.001002 
    Air = 1.825 * (10 ** (-5))

# 동점성 계수 ν = μ / ρ [m^2/s]
class Kinematic_Viscosity:
    PureWater = Absolute_Viscosity.PureWater / Density.Liquid.Water.Pure # 1.003e-06
    Air = Absolute_Viscosity.Air / Density.Gas.Air # 1.156e-5