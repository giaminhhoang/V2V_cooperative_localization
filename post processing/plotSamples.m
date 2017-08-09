function plotSamples(samples,color)

WAS_HOLD = ishold;

if ~WAS_HOLD
    hold on;
end

numSamples = size(samples, 1);

for s = 1:numSamples
    plotmarker(samples(s,1:2), color);
end

if ~WAS_HOLD
    hold off;
end
