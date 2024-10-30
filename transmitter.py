import tkinter as tk
from tkinter import filedialog, messagebox
import socket
import threading
import os

class TransmitterApp:
    def __init__(self, master):
        self.master = master
        master.title("Transmissor")

        self.label = tk.Label(master, text="Clique para selecionar uma imagem")
        self.label.pack(fill=tk.BOTH, expand=True)
        self.label.config(relief=tk.RAISED, borderwidth=2)
        self.label.bind("<Button-1>", self.browse_files)

        self.entry_ip = tk.Entry(master)
        self.entry_ip.pack()
        self.entry_ip.insert(0, "Endereço IP do receptor")

        self.file_path = None

    def browse_files(self, event=None):
        self.file_path = filedialog.askopenfilename(filetypes=[("Imagens", "*.png;*.jpg;*.gif;*.bmp")])
        if self.file_path:
            self.send_image()

    def send_image(self):
        receiver_ip = self.entry_ip.get()
        receiver_port = 5001

        if not receiver_ip or receiver_ip == "Endereço IP do receptor":
            messagebox.showerror("Erro", "Por favor, insira o endereço IP do receptor.")
            return

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((receiver_ip, receiver_port))
                with open(self.file_path, 'rb') as f:
                    while True:
                        data = f.read(4096)
                        if not data:
                            break
                        s.sendall(data)
            messagebox.showinfo("Sucesso", "Imagem enviada com sucesso!")
        except Exception as e:
            messagebox.showerror("Erro", f"Não foi possível enviar a imagem:\n{e}")

def main():
    root = tk.Tk()
    app = TransmitterApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
