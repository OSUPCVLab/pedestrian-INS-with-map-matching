function n = number_of_missed_samples(packetCount)
nCapturedviaXbee = length(packetCount);
n = 0; n1 = 0; n2 = 0;
for i=2:nCapturedviaXbee
    diff = packetCount(i) - packetCount(i-1);
    if (diff == 2 || diff == -254)
        fprintf('Sample number %i\n', i);
        fprintf('1 sample missed!!!\n');
        n = n + 1;
    else if (diff == 3 || diff == -253)
            fprintf('Sample number %i\n', i);
            fprintf('2 samples missed in a raw!!!\n');
            n1 = n1 + 1;
        else if (diff == 4 || diff == -252)
                fprintf('Sample number %i\n', i);
                fprintf('3 samples missed in a raw!!!');
                n2 = n2 + 1;
            end
        end
    end
end