/*This is the JavaScript of our HTML file which does all the functionality
that's available in our DashBoard*/
//Decalring Icons as a global variable
var greenIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowSize: [41, 41]
});
var redIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowSize: [41, 41]
});
var yellowIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-yellow.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
    iconSize: [25, 41],
    iconAnchor: [12, 41],
    popupAnchor: [1, -34],
    shadowSize: [41, 41]
});
var opt = {
            hAxis: { title: 'Order ID'},
            vAxis: { title: 'Time Taken (s)'},
            legend: { position: "none" },
        };
var opt1 = {
    curveType: 'function',
    hAxis: { title: 'Order ID'},
    vAxis: { title: 'Order, Dispatched and Shipped Time in (s)'},
    "tooltip": {"trigger": 'both'},
    legend: { position: "none" },
};
//Load Google Charts
google.charts.load('current', {'packages':['corechart']});
google.charts.setOnLoadCallback(refreshContents);
// Getting map from leaflet
var container = L.DomUtil.get('map');
if(container != null){
    container._leaflet_id = null;
}
// Setting View for the map
var map = L.map('map').setView([20.5937, 78.9629], 4);
/////////////Ajax Requests////////////
$(document).ready(function() {
    // Fetch the initial table
    refreshContents();
    // Fetch data from sheets every 5 second
    setInterval(refreshContents, 5000);
    });
    //function refreshContents
    function refreshContents(){
        var jsonDataObject =[];
        var graph_arr = [['Order ID', 'Time Taken', { role: 'style' }]];
        var graph_arr1 = [['Order ID', 'Order Time(s)', 'Dispatch Time(s)', 'Shipped Time(s)', { role: 'style' }]];
        var bar_color = [];
        $.getJSON('https://spreadsheets.google.com/feeds/list/1iiRZ1vmkAEJpX0HgNMvtuwO3jvm5qO0WP5JBmihzOLY/5/public/full?alt=json', function(data) {
        //add table to the dashboard
        var trHTML = '';
        for (var i = 0; i < data.feed.entry.length; ++i) {
            var myData_map, myData_order;
            var json_data = {
                "City": data.feed.entry[i].gsx$city.$t,
                "OrderID" : data.feed.entry[i].gsx$orderid.$t,
                "Item" : data.feed.entry[i].gsx$item.$t,
                "Priority": data.feed.entry[i].gsx$priority.$t,
                "Latitude": parseFloat(data.feed.entry[i].gsx$latitude.$t),
                "Longitude": parseFloat(data.feed.entry[i].gsx$longitude.$t),
                "OrderTime": data.feed.entry[i].gsx$orderdateandtime.$t,
                "Dispatchtime": data.feed.entry[i].gsx$dispatchdateandtime.$t,
                "Shippedtime": data.feed.entry[i].gsx$shippeddateandtime.$t,
                "DispatchStatus": data.feed.entry[i].gsx$orderdispatched.$t,
                "ShippedStatus": data.feed.entry[i].gsx$ordershipped.$t,
                "TimeTaken": parseFloat(data.feed.entry[i].gsx$timetaken.$t),
                "Cost": parseFloat(data.feed.entry[i].gsx$cost.$t)
            };
            jsonDataObject.push(json_data);
            trHTML += '<tr><td>' + json_data.OrderID +
                      '</td><td>' + json_data.Item +
                      '</td><td>' + json_data.Priority +
                      '</td><td>' + json_data.City +
                      '</td><td>'  + json_data.DispatchStatus +
                      '</td><td>'  + json_data.ShippedStatus +
                      '</td><td>'  + json_data.OrderTime +
                      '</td><td>'  + json_data.Dispatchtime +
                      '</td><td>'  + json_data.Shippedtime +
                      '</td><td>'  + json_data.TimeTaken +
                      '</td></tr>';
        }
        $('#tableContent').html(trHTML);
        var trHTML = '';

        // Setting color for the coloumns of graph according to priority of items
        for(var j in jsonDataObject){
            if(jsonDataObject[j].Priority == 'HP'){
                var color =  '#FF0000';
            }
            else if(jsonDataObject[j].Priority == 'MP'){
                var color =  '#FFFF00';
            }
            else if(jsonDataObject[j].Priority == 'LP'){
                var color =  '#00FF00';
            }
            bar_color.push(color)
        }
        // Converting Json Object to JavaScript Array
        for(var j in jsonDataObject){
            graph_arr.push([jsonDataObject[j].OrderID,jsonDataObject[j].TimeTaken, bar_color[j]]);
            if(jsonDataObject[j].Dispatchtime && jsonDataObject[j].Shippedtime) {
                var n1 = jsonDataObject[j].OrderTime.split(' ');
                var a1 = n1[1].split(':');
                var sec1 = (+a1[0]) * 60 * 60 + (+a1[1]) * 60 + (+a1[2]);
                var n2 = jsonDataObject[j].Dispatchtime.split(" ");
                var a2 = n2[1].split(':');
                var sec2 = (+a2[0]) * 60 * 60 + (+a2[1]) * 60 + (+a2[2]);
                var n3 = jsonDataObject[j].Shippedtime.split(' ');
                var a3 = n3[1].split(':');
                var sec3 = (+a3[0]) * 60 * 60 + (+a3[1]) * 60 + (+a3[2]);
                graph_arr1.push([jsonDataObject[j].OrderID,parseFloat(sec1), parseFloat(sec2), parseFloat(sec3), '#00FF00']);
                var graphArray_Final1 = google.visualization.arrayToDataTable(graph_arr1);
                var chart2 = new google.visualization.LineChart(document.getElementById('curve_chart'));
                chart2.draw(graphArray_Final1, opt1);
            }
        }
        var graphArray_Final = google.visualization.arrayToDataTable(graph_arr);
        var chart1 = new google.visualization.ColumnChart(document.getElementById('column_chart'));
        var data = new google.visualization.DataView(graphArray_Final);
        chart1.draw(data, opt);
        // add markers to the map
        for (var j = 0; j < jsonDataObject.length; j++) {
            if(jsonDataObject[j].DispatchStatus == 'YES' && jsonDataObject[j].ShippedStatus == 'YES')
                var icons = greenIcon
            else if(jsonDataObject[j].DispatchStatus == 'YES')
                var icons = yellowIcon
            else
                var icons = redIcon
            var marker = L.marker(L.latLng(parseFloat(jsonDataObject[j].Latitude), parseFloat(jsonDataObject[j].Longitude)),{icon: icons});
            // Attach the corresponding JSON data to your marker:
            marker.myJsonData =jsonDataObject[j];
            //function to display messages when marker is clicked
            function onClick_Marker(e) {
                    var marker = e.target;
                    if(marker.myJsonData.ShippedStatus == 'YES' && marker.myJsonData.DispatchStatus == 'YES') {
                        popup = L.popup()
                        .setLatLng(marker.getLatLng())
                        .setContent("Order ID: " + marker.myJsonData.OrderID + " || Dispatched: " +   marker.myJsonData.DispatchStatus + " || Shipped: " + marker.myJsonData.ShippedStatus)
                        .openOn(map);
                    }
                    else if(marker.myJsonData.DispatchStatus == 'YES') {
                        popup = L.popup()
                        .setLatLng(marker.getLatLng())
                        .setContent("Order ID: " + marker.myJsonData.OrderID + " || Dispatched: " +   marker.myJsonData.DispatchStatus + " || Shipped: NO")
                        .openOn(map);
                    }
                    else {
                        popup = L.popup()
                        .setLatLng(marker.getLatLng())
                        .setContent("Order ID: " + marker.myJsonData.OrderID + " || Dispatched: NO" + " || Shipped: NO")
                        .openOn(map);
                    }
                }
            marker.bindPopup(jsonDataObject[j].City, {
                    autoClose: false
            });
            map.addLayer(marker);
            marker.on('click', onClick_Marker)
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'
            }).addTo(map);
            }
            var a=0,b=0,c=0,s=0;
            for (var j = 0; j < jsonDataObject.length; j++)
            {
                if(jsonDataObject[j].OrderID)
                {
                    a=a+1;
                }
                if(jsonDataObject[j].DispatchStatus)
                {
                    b=b+1;
                }
                if(jsonDataObject[j].ShippedStatus)
                {
                    c=c+1;
                }
                s=s+jsonDataObject[j].Cost;
            }
            document.getElementById("a").innerHTML = a;
            document.getElementById("b").innerHTML = b;
            document.getElementById("c").innerHTML = c;
            document.getElementById("s").innerHTML = s;
        });
}