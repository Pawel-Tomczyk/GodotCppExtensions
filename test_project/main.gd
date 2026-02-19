extends Node

@onready var uart = $UartManager # Upewnij się, że węzeł tak się nazywa

func _ready():
	# Podpinamy sygnał
	uart.packet_received.connect(_on_packet_received)
	
	# Próbujemy otworzyć port (Zmień COM3 na swój port drona/mikrokontrolera)
	var success = uart.open_port("COM3", 115200)
	
	if success:
		print("Udało się otworzyć port UART z poziomu GDScript!")
		
		# Przykładowe wysłanie pakietu o ID 1 z danymi [10, 20, 30]
		var data_to_send = PackedByteArray([10, 20, 30])
		uart.send_packet(1, data_to_send)
	else:
		print("Błąd otwarcia portu.")

# Funkcja odpalana automatycznie przez C++, gdy przyjdzie poprawny pakiet
func _on_packet_received(id: int, data: PackedByteArray):
	print("Otrzymano pakiet! ID: ", id, " Dane: ", data)

func _exit_tree():
	# Pamiętaj, żeby zamknąć port przy zamykaniu gry!
	uart.close_port()
