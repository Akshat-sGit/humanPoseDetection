import hashlib

def proof_of_work(data, difficulty):
    nonce = 0
    while True:
        hashed_data = hashlib.sha256(f"{data}{nonce}".encode()).hexdigest()
        if hashed_data.startswith('0' * difficulty):
            return nonce, hashed_data
        nonce += 1

# Example usage:
if __name__ == "__main__":
    data_to_hash = input("Enter data to be hashed: ")
    difficulty_level = int(input("Enter difficulty level (number of leading zeroes): "))
    nonce, valid_hash = proof_of_work(data_to_hash, difficulty_level)
    print(f"Valid nonce: {nonce}")
    print(f"Valid hash: {valid_hash}")
