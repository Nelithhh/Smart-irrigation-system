$(document).ready(function(){                                       // --> rounding data into 2 decimal points
    function roundToTwoDecimal(num) {
        return Math.round((num + Number.EPSILON) * 100) / 100;
    }

    function fetchData() {                            // --> getting data to the website from the json
        const apiUrl = 'http://192.168.8.188/data'; 
        
        fetch(apiUrl)
        .then(response => response.json())
        .then(data => {
            document.getElementById('todayStatus').innerText = data.todayStatus;
            document.getElementById('rainValue').innerText = data.RainSensor;
            document.getElementById('moistureValue').innerText = roundToTwoDecimal(data.SoilMoistureSensor);
            document.getElementById('waterFlowValue').innerText = roundToTwoDecimal(data.FlowRate);
            document.getElementById('humidityValue').innerText = roundToTwoDecimal(data.Humidity);
            document.getElementById('pressureValue').innerText = roundToTwoDecimal(data.Pressure);
            document.getElementById('temperatureValue').innerText = roundToTwoDecimal(data.Temperature);
            document.getElementById('dailyWater').innerText = roundToTwoDecimal(data.dailyWaterUsage);
            document.getElementById('tomorrowPrediction').innerText = data.tomorrowPrediction;
            document.getElementById('monthlyPrediction').innerText = data.monthlyPrediction;
        })
        .catch(error => console.error('Error fetching data:', error));
    }

    fetchData();
    setInterval(fetchData, 1000);  

    $('#dataForm').submit(function(event) {                                   // --> sending data to the nodemcu
        event.preventDefault();
        const formData = $(this).serialize();
        $.post('http://192.168.8.188/trigger', formData)
            .done(function(response) {
                console.log('Data submitted successfully:', response);
                $('#submissionStatus').text('Data submitted successfully.');

            })
            .fail(function(error) {
                console.error('Error submitting data:', error);
                $('#submissionStatus').text('Error submitting data. Please try again.');
            });
    });
});
