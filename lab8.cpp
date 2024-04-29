#include <iostream>
#include <cmath>

// Function to calculate the block reward
double calculateBlockReward(int height) {
    int halvings = height / 210000; // Bitcoin halves the reward every 210,000 blocks
    double reward = 50.0; // Initial block reward
    
    // Apply halving
    for (int i = 0; i < halvings; ++i) {
        reward /= 2;
    }
    
    return reward;
}

int main() {
    int blockHeight = 680000; // Example block height
    
    double reward = calculateBlockReward(blockHeight);
    
    std::cout << "Block Reward at height " << blockHeight << ": " << reward << " BTC" << std::endl;
    
    return 0;
}
