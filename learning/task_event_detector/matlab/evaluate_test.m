clear all;
close all;

load /tmp/training_label.txt;
load /tmp/training_data.txt;

load /tmp/predicted_labels.txt;
load /tmp/test_data.txt;

[cols rows] = size(training_data);
if(cols ~= length(training_label))
   error('Number of cols must equal number of labels');
end

markersize = 12;

fullscreen = get(0,'ScreenSize');
figure('Position',[0 -50 fullscreen(3) fullscreen(4)])

hold on;
box on;
for i=1:cols
    if(training_label(i) > 0)
        plot(training_data(i,1), training_data(i,2), 'go', 'markersize', markersize);
    else
        plot(training_data(i,1), training_data(i,2), 'gx', 'markersize', markersize);
    end    
end

[cols rows] = size(test_data);
if(cols ~= length(predicted_labels))
   error('Number of cols must equal number of labels');
end

for i=1:cols
    if(predicted_labels(i) > 0)
        plot(test_data(i,1), test_data(i,2), 'bo', 'markersize', markersize);
    else
        plot(test_data(i,1), test_data(i,2), 'bx', 'markersize', markersize);
    end    
end

hold off;

%keyboard;

