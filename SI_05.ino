#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;

#define SI5351_I2C_ADDR 0x60
#define RTTY_PIN        25
#define SI_POWER_PIN    16
#define DS_POWER_PIN    29
#define SI_PIN_SDA      18
#define SI_PIN_SCL      19
#define DS_PIN_SDA      20
#define DS_PIN_SCL      21
//uint32_t Mark_freq  = 10000000;
//uint32_t Space_freq = 10000000;

uint32_t Mark_freq  = 3590830;
uint32_t Space_freq = 3591000;
uint64_t freq_10MHz = 10001067;
uint64_t Xtal_freq  = 25000000 * freq_10MHz / 10000000;

const uint16_t BIT_TIME = 22;

#define MTK2_LAT 0x1F // 000 11111
#define MTK2_FIG 0x1B // 000 11011
#define MTK2_RUS 0x00 // 000 00000

enum TransmitterState {MARK, SPACE, STOP};
TransmitterState Si_State = STOP;

const uint8_t INIT_REGS[] = {0x03, 0x10, 0x11, 0x12, 0xB7, 0x95, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0xB1, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x10, 0x03};
const uint8_t INIT_DATA[] = {0xFF, 0x80, 0x80, 0x80, 0xC0, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA8, 0xE8, 0x00, 0x7B, 0xAB, 0xD4, 0x44, 0x08, 0x0F, 0x00};

const uint8_t FREQ_REGS[]  = {0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x10, 0x03}; 
const uint8_t DATA_STOP[]  = {0xB2, 0xD6, 0x00, 0x7B, 0x53, 0xDB, 0xA9, 0x1E, 0x80, 0xFF};
uint8_t DATA_MARK[10];
uint8_t DATA_SPACE[10];

typedef struct {
    const char* s;
    uint8_t code;
} mtk2_map_t;

// Латинский регистр
const mtk2_map_t table_lat[] = {
    {"A", 0x03}, {"B", 0x19}, {"C", 0x0E}, {"D", 0x09}, {"E", 0x01}, 
    {"F", 0x0D}, {"G", 0x1A}, {"H", 0x14}, {"I", 0x06}, {"J", 0x0B}, 
    {"K", 0x0F}, {"L", 0x12}, {"M", 0x1C}, {"N", 0x0C}, {"O", 0x18}, 
    {"P", 0x16}, {"Q", 0x17}, {"R", 0x0A}, {"S", 0x05}, {"T", 0x10}, 
    {"U", 0x07}, {"V", 0x1E}, {"W", 0x13}, {"X", 0x1D}, {"Y", 0x15}, 
    {"Z", 0x11},
    {"a", 0x03}, {"b", 0x19}, {"c", 0x0E}, {"d", 0x09}, {"e", 0x01}, 
    {"f", 0x0D}, {"g", 0x1A}, {"h", 0x14}, {"i", 0x06}, {"j", 0x0B}, 
    {"k", 0x0F}, {"l", 0x12}, {"m", 0x1C}, {"n", 0x0C}, {"o", 0x18}, 
    {"p", 0x16}, {"q", 0x17}, {"r", 0x0A}, {"s", 0x05}, {"t", 0x10}, 
    {"u", 0x07}, {"v", 0x1E}, {"w", 0x13}, {"x", 0x1D}, {"y", 0x15}, 
    {"z", 0x11},
    {" ", 0x04}, {"\n", 0x08}, {"\r", 0x02}, {NULL, 0}
};

// Цифровой регистр (ФИГ)
const mtk2_map_t table_fig[] = {
    {"-", 0x03}, {"?", 0x19}, {":", 0x0E}, {"3", 0x01}, {"Э", 0x0D},
    {"Ш", 0x1A}, {"Щ", 0x14}, {"8", 0x06}, {"Ю", 0x0B}, {"(", 0x0F},
    {")", 0x12}, {".", 0x1C}, {",", 0x0C}, {"9", 0x18}, {"0", 0x16},
    {"1", 0x17}, {"4", 0x0A}, {"'", 0x05}, {"5", 0x10}, {"7", 0x07},
    {"=", 0x1E}, {"2", 0x13}, {"/", 0x1D}, {"6", 0x15}, {"+", 0x11},
    {"э", 0x0D}, {"ш", 0x1A}, {"щ", 0x14}, {"ю", 0x0B}, {"Ч", 0x0A},
    {"ч", 0x0A},
    {NULL, 0}
};

// Русский регистр (РУС)
const mtk2_map_t table_rus[] = {
    {"А", 0x03}, {"Б", 0x19}, {"Ц", 0x0E}, {"Д", 0x09}, {"Е", 0x01}, 
    {"Ф", 0x0D}, {"Г", 0x1A}, {"Х", 0x14}, {"И", 0x06}, {"Й", 0x0B}, 
    {"К", 0x0F}, {"Л", 0x12}, {"М", 0x1C}, {"Н", 0x0C}, {"О", 0x18}, 
    {"П", 0x16}, {"Я", 0x17}, {"Р", 0x0A}, {"С", 0x05}, {"Т", 0x10}, 
    {"У", 0x07}, {"Ж", 0x1E}, {"В", 0x13}, {"Ь", 0x1D}, {"Ы", 0x15}, 
    {"З", 0x11}, {"Ъ", 0x1D}, {"Ё", 0x01},
    {"а", 0x03}, {"б", 0x19}, {"ц", 0x0E}, {"д", 0x09}, {"е", 0x01}, 
    {"ф", 0x0D}, {"г", 0x1A}, {"х", 0x14}, {"и", 0x06}, {"й", 0x0B}, 
    {"к", 0x0F}, {"л", 0x12}, {"м", 0x1C}, {"н", 0x0C}, {"о", 0x18}, 
    {"п", 0x16}, {"я", 0x17}, {"р", 0x0A}, {"с", 0x05}, {"т", 0x10}, 
    {"у", 0x07}, {"ж", 0x1E}, {"в", 0x13}, {"ь", 0x1D}, {"ы", 0x15}, 
    {"з", 0x11}, {"ъ", 0x1D}, {"ё", 0x01},
    {NULL, 0}
};

enum MTK2_STATE { LAT, FIG, RUS };
MTK2_STATE current_reg = LAT;

void calculate_freq_bytes(uint32_t freq_hz, uint8_t* out_data) {
  // частоты от 1 до 160 МГц
    const uint32_t pll_multiplier = 36;
    uint64_t f_pll = (uint64_t)Xtal_freq * pll_multiplier; 
    uint32_t divider_int = (uint32_t)(f_pll / freq_hz);
    uint32_t remainder = (uint32_t)(f_pll % freq_hz);
    uint32_t ms_den = 1048575;
    uint32_t ms_num = (uint32_t)(((uint64_t)remainder * ms_den) / freq_hz);
    uint32_t p1 = (uint32_t)(128 * divider_int + ((128 * ms_num) / ms_den) - 512);
    uint32_t p2 = (uint32_t)(128 * ms_num - ms_den * ((128 * ms_num) / ms_den));
    uint32_t p3 = ms_den;
    out_data[0] = (p3 >> 8) & 0xFF;
    out_data[1] = p3 & 0xFF;
    out_data[2] = (p1 >> 16) & 0x03;
    out_data[3] = (p1 >> 8) & 0xFF;
    out_data[4] = p1 & 0xFF;
    out_data[5] = ((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F);
    out_data[6] = (p2 >> 8) & 0xFF;
    out_data[7] = p2 & 0xFF;
    out_data[8] = 0x0F;
    out_data[9] = 0x00;
}

bool si5351_write_reg(uint8_t reg, uint8_t data) {
  for (int i = 0; i < 3; i++) {
    Wire1.beginTransmission(SI5351_I2C_ADDR);
    Wire1.write(reg);
    Wire1.write(data);
    if (Wire1.endTransmission() == 0) {
      return true; 
    }
    if (i < 2) {
      Serial.print("SI5351: Retrying I2C... Attempt ");
      Serial.println(i + 2);
      Wire1.begin();
      delay(10);
    }
  }
  return false;
}

void set_freq(TransmitterState Si_State) {
  const uint8_t* data_ptr;
  switch (Si_State) {
    case MARK:  data_ptr = DATA_MARK;  digitalWrite(RTTY_PIN, HIGH); break;
    case SPACE: data_ptr = DATA_SPACE; digitalWrite(RTTY_PIN, LOW);  break;
    case STOP:  data_ptr = DATA_STOP;  digitalWrite(RTTY_PIN, LOW);  break;
    default: return;
  }
  for (int i = 0; i < 10; i++) {
    si5351_write_reg(FREQ_REGS[i], data_ptr[i]);
  }
}

void SI_POWER_ON() {
    digitalWrite(DS_POWER_PIN, HIGH);
    digitalWrite(SI_POWER_PIN, LOW);
    calculate_freq_bytes(Mark_freq, DATA_MARK);
    calculate_freq_bytes(Space_freq, DATA_SPACE);
    delay(100);
    Wire1.begin();
  if (si5351_write_reg(INIT_REGS[0], INIT_DATA[0])) {
    Serial.println("SI5351: OK");
  } else {
    Serial.println("SI5351: NOT FOUND!");
    return;
  }
  for (int i = 1; i < 25; i++) {
    if (!si5351_write_reg(INIT_REGS[i], INIT_DATA[i])) {
      Serial.print("SI5351: Failed at reg ");
      Serial.println(INIT_REGS[i], HEX);
    }
  }
  set_freq(STOP);
    if (rtc.begin(&Wire)) {
        Serial.println("RTC DS3231: OK");
    } else {
        Serial.println("RTC DS3231: NOT FOUND!");
    }
}

void SI_POWER_OFF() {
    digitalWrite(SI_POWER_PIN, HIGH);
    digitalWrite(DS_POWER_PIN, LOW);
}

bool find_in_table(const mtk2_map_t* table, const char* s, uint8_t &code, int &len) {
    for (int i = 0; table[i].s != NULL; i++) {
        int l = strlen(table[i].s);
        if (strncmp(table[i].s, s, l) == 0) {
            code = table[i].code;
            len = l;
            Serial.print(table[i].s);
            return true;
        }
    }
    return false;
}

void send_rtty_bit(TransmitterState state) {
    set_freq(state);
    delay(BIT_TIME);
}

void send_rtty_code(uint8_t code) {
    send_rtty_bit(SPACE);
    for (int i = 0; i < 5; i++) {
        if (code & (1 << i)) send_rtty_bit(MARK);
        else send_rtty_bit(SPACE);
    }
    set_freq(MARK);
    delay(BIT_TIME * 1.5); 
}

void send_rtty_string(const char* s) {
    while (*s) {
        uint8_t code = 0;
        int char_len = 1;
        MTK2_STATE target_reg = current_reg;
        bool found = false;

        if (find_in_table(table_lat, s, code, char_len)) { target_reg = LAT; found = true; }
        else if (find_in_table(table_fig, s, code, char_len)) { target_reg = FIG; found = true; }
        else if (find_in_table(table_rus, s, code, char_len)) { target_reg = RUS; found = true; }

        if (found) {
            // Смена регистра
            if (target_reg != current_reg) {
                uint8_t reg_code = (target_reg == LAT) ? MTK2_LAT : (target_reg == FIG ? MTK2_FIG : MTK2_RUS);
                send_rtty_code(reg_code);
                current_reg = target_reg;
            }
            // Отправка символа
            send_rtty_code(code);
            s += char_len;
        } else {
            s++;
        }
    }
    send_rtty_code(0x02);
    send_rtty_code(0x08);
    Serial.println("");
}

void send_current_time() {
    if (!rtc.begin(&Wire)) {
        send_rtty_string("RTC DS3231: ERROR");
        return;
    }
    
    DateTime now = rtc.now();
    float temp = rtc.getTemperature();
    char buffer[64];
    int temp_int = (int)temp;
    int temp_frac = (int)((temp - temp_int) * 10);
    sprintf(buffer, "TIME: %02d:%02d:%02d %02d/%02d/%d TEMP: %d.%dC", 
            now.hour(), now.minute(), now.second(), 
            now.day(), now.month(), now.year(),
            temp_int, abs(temp_frac));
    
    send_rtty_string(buffer);
}


void setup() {
    pinMode(RTTY_PIN, OUTPUT);
    pinMode(DS_POWER_PIN, OUTPUT);
    pinMode(SI_POWER_PIN, OUTPUT);
    Wire1.setSDA(SI_PIN_SDA);
    Wire1.setSCL(SI_PIN_SCL);
    Wire1.begin();
    Wire1.setClock(400000);
    Wire.setSDA(DS_PIN_SDA);
    Wire.setSCL(DS_PIN_SCL);
    Wire.begin();
    Serial.begin(115200);
    if (!rtc.begin(&Wire)) {
        Serial.println("RTC DS3231: NOT FOUND");
    }
}

void loop() {
    Serial.println("Power ON");
    SI_POWER_ON();
    set_freq(MARK);
    delay(500);
    send_rtty_string("CQ CQ CQ DE RU0AOG ПРИВЕТ");
    send_current_time();
    send_rtty_string("Съешь ещё этих мягких французских булок, да выпей же чаю (русский)");
    send_rtty_string("The quick brown fox jumps over the lazy dog (english)");
    send_rtty_string("0123456789,+/-.=(:'?)");
    set_freq(MARK);
    delay(500);
    set_freq(STOP);
    SI_POWER_OFF();
    Serial.println("Power OFF");
    delay(5000);
}