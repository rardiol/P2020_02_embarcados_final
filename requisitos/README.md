# Controle de Elevador

## Requisitos Funcionais

* Caso um botão interno seja apertado, ele deve ser adicionado a lista de destinos ativos do elevador.
* Apertar um botão interno que já é um destino ativo não tem efeito.
* Cada botão interno do elevador deve estar aceso se e apenas se ele for um destino ativo desse elevador.
* Apos abrir as portas, o elevador deve permanecer com ela aberta por pelo menos 5 segundos.
  * Apos esses 5 segundos, se houver um destino ativo, o elevador deve fechar as portas e se deslocar. 
* Se um elevador não tiver nenhum destino ativo, ele deve permanecer no mesmo andar com as portas abertas.
* Se um elevador estiver com a porta aberta, esse andar deixa de ser um destino ativo desse elevador.
* Os elevadores devem parar apenas nos andares corretos e alinhados.
* Os elevadores devem parar apenas em seus destinos ativos.
* Quando um botão externo é apertado, o sistema deve designar esse andar como destino ativo de um elevador.
  * Essa designação deve ter o objetivo de melhorar a eficiência do sistema, preferindo manter elevadores em movimento na mesma direção.



## Requisitos Não Funcionais

* Todos elevadores começam no térreo com portas fechadas e com destino ao térreo.
* Sistema de controle roda em uma Tiva
* Comunicação com o simulador usando porta serial
* Velocidade de comunicação 115200
* Há uma tarefa de controle da UART.
* O sistema opera utilizando mensagens.
* Todos os botoes de subir e todos os botoes de descer do mesmo andar são equivalentes.
* Há uma tarefa de atribuição de destinos dos elevadores.
* Há uma tarefas individual de controle para cada elevator
* O elevador só se descola com porta fechada.
* Atribuir um destino como ativo mais de uma vez não tem efeito
