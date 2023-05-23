# AutoHidroTech
Bem-vindo ao repositório do projeto Autohidrotech! Este projeto tem como objetivo automatizar e simplificar o cultivo de hortas hidropônicas utilizando o microcontrolador ESP32. Com a automação de tarefas como monitoramento de pH, condutividade elétrica, TDS e controle de irrigação, você poderá desfrutar de uma horta eficiente e de fácil manutenção.
# Recursos 
* Monitoramento em tempo real do pH, TDS e da condutividade elétrica da solução nutritiva.
* Controle automático da irrigação a cada 15 minutos ou acionamento manual.
* Visualização dos dados em um display LCD e também remotamente em um aplicativo móvel(Blynk).
* Integração com a plataforma de nuvem Blynk Cloud para armazenamento e análise de dados.
# Lista de Materiais
1. ESP32
2. ESP32-DevKitC-32
3. Módulo DFRobot PH Meter
4. Módulo TDS Meter
5. Display LCD I2C
6. Módulo Relé 2 canais 5v
7. Conector P4 macho/femea
8. Bomba d'água p/ aquário 
9. Fonte 5v/12v 1A
10. Case impresso em 3D
# Instalação 
1. Clone este repositório para o seu ambiente local.
2. Conecte o ESP32 ao seu computador.
3. Abra o projeto no Arduino IDE.
4. Instale as bibliotecas necessárias.
   * LiquidCrystal_I2C e Wire - para o displayLCD
   * Blynk - realizar a comunicação com o servidor Blynk.
   * DFRobot_PH Library - para o sensor PH
6. Faça o upload do código para o ESP32.
7. Realize as conexões adequadas entre o ESP32, sensores e atuadores conforme o esquema fornecido.
8. Configure as credenciais de rede Wi-Fi e as preferências de configuração no código.
# Uso 
1.Ligue o sistema e aguarde a inicialização.
2.Acompanhe as informações no display LCD ou acesse o aplicativo móvel (Blynk) para visualizar os dados remotamente.
3.Ajuste as configurações conforme necessário.
4.Monitore regularmente os níveis de pH, condutividade elétrica e TDS.
5.Aproveite uma horta hidropônica automatizada e de alto desempenho!
# Contribuição
Contribuições são bem-vindas! Sinta-se à vontade para abrir problemas, enviar solicitações de pull ou fornecer melhorias para o projeto. 

