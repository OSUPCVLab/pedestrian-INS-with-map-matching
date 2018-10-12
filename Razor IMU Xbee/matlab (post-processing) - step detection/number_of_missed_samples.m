function n = number_of_missed_samples(packetCount)
nCapturedviaXbee = length(packetCount);
n = 0;
for i=2:nCapturedviaXbee
    diff = packetCount(i) - packetCount(i-1);
    if (diff == 2 || diff == -254)
        n = n + 1;
    end
end