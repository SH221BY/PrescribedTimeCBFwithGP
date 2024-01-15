function noise = GenerateGaussiaNoise( meanValue, std_deviation, noise_length)
    noise = meanValue + std_deviation * randn(noise_length, 1);
end