import csv
import os

class FileManager:
    def __init__(self, save_dir="../saved_paths"):
        self.save_dir = os.path.join(os.path.dirname(__file__), save_dir)
        os.makedirs(self.save_dir, exist_ok=True)  # Crear la carpeta si no existe

    def save_path(self, path_points, filename="trayectoria.csv"):
        file_path = os.path.join(self.save_dir, filename)
        with open(file_path, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y"])
            writer.writerows(path_points)
        print(f"Trayectoria guardada en {file_path}")

    def load_path(self, filename="trayectoria.csv"):
        file_path = os.path.join(self.save_dir, filename)
        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                reader = csv.reader(file)
                next(reader)
                path_points = [(float(row[0]), float(row[1])) for row in reader]
            print(f"Trayectoria cargada desde {file_path}")
            return path_points
        else:
            print(f"No se encontró {file_path}.")
            return []
