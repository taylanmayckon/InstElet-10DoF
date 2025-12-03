<div align="center">
  <a href="./">
    <img src="https://img.shields.io/badge/Firmware-ESP32-blue?style=for-the-badge&logo=espressif" alt="Firmware ESP32">
  </a>
  <a href="https://github.com/Priisciila/Dashboard-Instrumentacao-Eletronica-">
    <img src="https://img.shields.io/badge/Frontend-Dashboard-green?style=for-the-badge&logo=react" alt="Dashboard Frontend">
  </a>
</div>

<br />

# 📡 InstElet-10DoF

**Sistema de Telemetria e Fusão Sensorial 10DoF com ESP32**

> Este projeto foi desenvolvido como trabalho da disciplina de **Instrumentação Eletrônica** do curso de Engenharia de Computação da **UNIVASF**.

O sistema realiza a leitura, fusão e filtragem de dados de um módulo IMU 10DoF (Acelerômetro, Giroscópio, Magnetômetro e Barômetro), transmitindo a atitude e dados ambientais via Wi-Fi.

## 📌 Visão Geral

O **InstElet-10DoF** utiliza o framework **ESP-IDF** para processar dados brutos de sensores e calcular a orientação espacial do dispositivo em tempo real.

### Principais Funcionalidades
* **📐 Leitura de Sensores:** Aceleração ($g$), Velocidade Angular ($^\circ/s$) e Campo Magnético ($gauss$).
* **🌡️ Dados Ambientais:** Altitude e Pressão Atmosférica (via BMP180).
* **🔄 Fusão Sensorial:** Cálculo de orientação (*Pitch, Roll, Yaw*) utilizando **Filtro de Madgwick**.
* **📡 Telemetria IoT:** Envio de pacotes JSON via HTTP POST para um servidor remoto.
* **📉 Filtragem Digital:** Uso da biblioteca `espp/filters` para suavização de dados.

### Aplicações
* Controle de atitude (Drones/VANTs)
* Robótica móvel
* Sistemas de navegação inercial
* Análise de vibração e movimento

---

## 🛠️ Hardware Necessário

* **Microcontrolador:** ESP32 (DevKit v1 ou similar)
* **Módulo IMU 10DoF (GY-87):**
    * MPU6050 (Acelerômetro + Giroscópio)
    * HMC5883L (Magnetômetro)
    * BMP180 (Barômetro/Termômetro)
* **Conexão:** Protocolo I2C

---

## 🖥️ Dashboard & Visualização

Para visualizar a telemetria em tempo real (gráficos e indicadores), este projeto deve ser utilizado em conjunto com o Front-end desenvolvido para o projeto.

O dashboard recebe os pacotes JSON via HTTP e exibe as informações.

🔗 **Acesse o repositório do Dashboard:**
[**Dashboard-Instrumentacao-Eletronica**](https://github.com/Priisciila/Dashboard-Instrumentacao-Eletronica-)

---

## ⚙️ Dependências e Software

Este projeto é baseado no **ESP-IDF v5.x**.

**Componentes Externos:**
Utilizamos o componente `espp/filters` do *ESP Component Registry* para a implementação dos filtros digitais.

Para instalar as dependências manualmente (caso não ocorra automaticamente no build):

```bash
idf.py add-dependency "espp/filters^1.0.31"
```

---

## 🚀 Como Executar

### 1. Clonar o Repositório
```bash
git clone [https://github.com/taylanmayckon/InstElet-10DoF](https://github.com/taylanmayckon/InstElet-10DoF)
cd InstElet-10DoF
```

### 2. Configurar e Rodar o Servidor (Dashboard)
Antes de gravar o código no ESP32, é necessário que o servidor esteja rodando para receber os dados.

1. Acesse o repositório do Front-end: [**Link para o Dashboard**](https://github.com/Priisciila/Dashboard-Instrumentacao-Eletronica-)
2. Siga as instruções de instalação e inicie o servidor.
3. Anote o **IP da sua máquina** na rede local.

### 3. Configurar Credenciais (Wi-Fi e Servidor)
> ⚠️ Importante: Atualmente, as credenciais de Wi-Fi e o IP do servidor estão definidos via código.
Abra o arquivo main/main.cpp e edite as seguintes linhas com sua configuração de rede:
```c
#define WIFI_SSID "SEU_SSID_AQUI"
#define WIFI_PASSWORD "SUA_SENHA_AQUI"
#define SERVER_IP "http://SEU_IP_DO_SERVIDOR:PORTA"
```

### 4. Compilar e Gravar
Conecte o ESP32 via USB e execute:
```bash
# Configurar o target (se necessário)
idf.py set-target esp32

# Compilar o projeto
idf.py build

# Gravar no microcontrolador
# (Substitua PORTA pela sua porta serial, ex: COM3 no Windows ou /dev/ttyUSB0 no Linux)
idf.py -p PORTA flash
```

### 5. Monitorar Saida
Para visualizar os logs em tempo real:
```bash
idf.py monitor
```

---

## 📊 Formato dos Dados (JSON)
```json
{
  "acelerometro": { "x": -0.98, "y": 0.02, "z": -0.15 },
  "giroscopio": { "x": -0.25, "y": -1.10, "z": -0.05 },
  "magnetometro": { "x": -0.17, "y": -0.07, "z": 0.03 },
  "bmp180": { "temperatura": 26.5, "pressao": 984.0 },
  "orientacao": { "pitch": 178.7, "roll": 82.8, "yaw": 104.4, "altitude": 0.0 }
}
```

---

## 📁 Estrutura do Projeto
```plaintext
InstElet-10DoF/
├── components/                  # Drivers dos sensores (BMP180, HMC5883L, MPU6050, WiFi)
├── main/
│   ├── main.cpp                 # Task principal e loop de controle
│   └── idf_component.yml        # Gerenciador de dependências
├── CMakeLists.txt               # Configuração de build do CMake
└── sdkconfig                    # Configurações do ESP-IDF
```

---

## 👨‍💻 Desenvolvido por

| Nome | GitHub |
| :--- | :--- |
| **Gabriel Menezes** | [@GabrielMenezesCarvalho](https://github.com/GabrielMenezesCarvalho) |
| **Nivaldo Guedes** | [@nivaldoguedes](https://github.com/nivaldoguedes) |
| **Priscila Araújo** | [@Priisciila](https://github.com/Priisciila) |
| **Taylan Mayckon** | [@taylanmayckon](https://github.com/taylanmayckon) |







