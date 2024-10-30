import tkinter as tk
from tkinter import filedialog, messagebox
import socket
import threading
import os

class ReceiverApp:
    def __init__(self, master):
        self.master = master
        master.title("Receptor")

        self.label = tk.Label(master, text="Aguardando imagem...")
        self.label.pack(fill=tk.BOTH, expand=True)
        self.label.config(relief=tk.RAISED, borderwidth=2)

        self.button_save = tk.Button(master, text="Escolher pasta de destino", command=self.choose_directory)
        self.button_save.pack()

        self.save_directory = os.getcwd()

        self.start_server()

    def choose_directory(self):
        self.save_directory = filedialog.askdirectory()
        if not self.save_directory:
            self.save_directory = os.getcwd()

    def start_server(self):
        server_thread = threading.Thread(target=self.server_thread)
        server_thread.daemon = True
        server_thread.start()

    def server_thread(self):
        host = ''
        port = 5001

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            while True:
                conn, addr = s.accept()
                threading.Thread(target=self.handle_client, args=(conn, addr)).start()

    def handle_client(self, conn, addr):
        with conn:
            print('Conectado por', addr)
            data = b''
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
            if data:
                file_path = os.path.join(self.save_directory, 'imagem_recebida')
                # Ensure unique filename
                count = 1
                base_file_path = file_path
                while os.path.exists(file_path):
                    file_path = f"{base_file_path}_{count}"
                    count += 1
                with open(file_path, 'wb') as f:
                    f.write(data)
                self.label.config(text=f"Imagem recebida de {addr}")
                messagebox.showinfo("Imagem Recebida", f"Imagem salva em {file_path}")

def main():
    root = tk.Tk()
    app = ReceiverApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
