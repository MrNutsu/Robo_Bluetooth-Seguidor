# 🤖 ESP32 Line Follower Robot

Firmware para um robô seguidor de linha com controle **automático via PID** e **manual via Bluetooth**, desenvolvido com **ESP32**.

<div align="center">
  <img src="images/robot.jpg" alt="Robô Montado" width="400"/>
  
</div>

---

## ⚙️ Funcionalidades

- 🔁 **Modo Automático (PID)**: o robô segue uma linha preta em fundo branco usando sensores de refletância.
- 🎮 **Modo Manual (Bluetooth)**: controle total dos movimentos via comandos Bluetooth.
- 🎯 **Ajuste de PID em tempo real**: sintonize Kp, Ki e Kd sem reprogramar o robô.

---

## 📡 Comandos Bluetooth

| Comando | Ação                |
|---------|---------------------|
| `L`     | Liga/desliga o modo seguidor de linha |
| `F`     | Anda para frente     |
| `B`     | Ré                   |
| `R`     | Gira para a direita  |
| `L`     | Gira para a esquerda |
| `G`     | Frente-esquerda      |
| `I`     | Frente-direita       |
| `H`     | Ré-esquerda          |
| `J`     | Ré-direita           |
| `S`     | Parar                |
| `0–9`, `q` | Ajusta velocidade (10% a 100%) |

### 🎚 Ajuste de PID

Envie **6 bytes sequenciais** para definir os valores de Kp, Ki e Kd:

| Byte | Valor     | Exemplo (`Kp = 1.2`) |
|------|-----------|----------------------|
| 1    | `Kp`      | `12`                 |
| 2    | Potência de 10 negativa (expoente) | `1` (para 10⁻¹) |
| 3–4  | `Ki` + expoente |
| 5–6  | `Kd` + expoente |

---

## 🔌 Hardware Utilizado

- ESP32 DevKit
- Sensor de refletância **QTR-5-RC**
- Ponte H **L298N**
- 2 motores DC
- Rodas e chassi de robô
- Fonte de energia (ex: bateria Li-ion 7.4V)
- Jumpers e protoboard

<div align="center">
  <img src="images/circuit.jpg" alt="Esquema do Circuito" width="400"/>
</div>

---

## 🧠 Código

O código principal está no arquivo [`esp32_line_follower.ino`](esp32_line_follower.ino).

### 📌 Observações:

- A calibração dos sensores ocorre nos **10 primeiros segundos** após ligar.
- A prioridade de execução é: **modo manual quando Bluetooth está conectado**, senão, entra no **modo automático**.

---

## 🚀 Como Usar

1. Faça upload do firmware para sua ESP32 via Arduino IDE.
2. Emparelhe com o ESP32 usando um app Bluetooth Serial (ex: Serial Bluetooth Terminal).
3. Envie comandos via Bluetooth.
4. (Opcional) Faça o tuning dos parâmetros PID em tempo real.

---

## 📄 Licença

Distribuído sob a licença MIT. Sinta-se livre para modificar e melhorar!

---

