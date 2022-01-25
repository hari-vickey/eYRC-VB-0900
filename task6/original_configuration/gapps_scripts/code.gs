// Function doGet
function doGet(e){
    // This function will get all the incoming parameters
    var ss = SpreadsheetApp.getActive();
    var sheet = ss.getSheetByName(e.parameter["id"]);
    var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
    var lastRow = sheet.getLastRow();
    var cell = sheet.getRange('a1');
    var col = 0;
    var d = new Date();
    var addedTime = Utilities.formatDate(d, SpreadsheetApp.getActive().getSpreadsheetTimeZone(), "EEE MMM d yyyy, hh:mm:ss");
    for (i in headers){
      // loop through the headers and if a parameter name matches the header name insert the value
        if (headers[i] == "Timestamp")
        {
             val = addedTime
        }
        else
        {
            val = e.parameter[headers[i]];
        }
        // append data to the last row
        cell.offset(lastRow, col).setValue(val);
        col++;
    }
    // To send email after the package OrdersDispatched
    if(e.parameter["id"] == "OrdersDispatched")
    {
        var dispLastRow = sheet.getLastRow();
        var data = sheet.getRange(dispLastRow,10).getValues();
        var order_id = sheet.getRange(dispLastRow,4).getValues();
        var item = sheet.getRange(dispLastRow,6).getValues();
        var quan = sheet.getRange(dispLastRow,8).getValues();
        var city = sheet.getRange(dispLastRow,5).getValues();
        var dispDate = sheet.getRange(dispLastRow,1).getValues();
        var cost = sheet.getRange(dispLastRow,9).getValues();
        var to = "eyrc.vb.0900@gmail.com";
        var eto = "eyrc.vb.0000@gmail.com";
        if(data == "YES")
        {
            var message = "Hello!\n\nYour order has been dispatched,contact us if you have any questions.We are here to help you. \n\nORDER SUMMARY :   \n\nOrder Number : "+order_id+"\nItem : "+item+"\nQuantity : "+quan+"\nDispatch Date and Time : "+dispDate+"\nCity : "+city+"\nCost : "+cost+"\nTHANK YOU!";

            MailApp.sendEmail(to, " Your Order is Dispatched! ", message);
            MailApp.sendEmail(eto, " Your Order is Dispatched! ", message);
        }
    }
    // To send email after the package OrdersShipped
    if(e.parameter["id"] == "OrdersShipped")
    {
        var dispLastRow = sheet.getLastRow();
        var data = sheet.getRange(dispLastRow,10).getValues();
        var order_id = sheet.getRange(dispLastRow,4).getValues();
        var item = sheet.getRange(dispLastRow,6).getValues();
        var quan = sheet.getRange(dispLastRow,8).getValues();
        var city = sheet.getRange(dispLastRow,5).getValues();
        var shipDate = sheet.getRange(dispLastRow,1).getValues();
        var cost = sheet.getRange(dispLastRow,9).getValues();
        var to = "eyrc.vb.0900@gmail.com";
        var eto = "eyrc.vb.0000@gmail.com";
        if(data == "YES")
        {
            if(e.parameter["Priority"] == "HP")
            {
                var date = 1;
                var est = d.setDate(d.getDate() + date);
            }
            else if(e.parameter["Priority"] == "MP")
            {
                var date = 3;
                var est = d.setDate(d.getDate() + date);
            }
            else
            {
                var date = 5;
                var est = d.setDate(d.getDate() + date);
            }
            var est = Utilities.formatDate(d, SpreadsheetApp.getActive().getSpreadsheetTimeZone(), "yyyy-MM-dd");
            var message = "Hello!\n\nYour order has been shipped.It will be drone delivered to you in " + date + " day,contact us if you have any questions.We are here to help you. \n\nORDER SUMMARY : \n\nOrder Number : "+order_id+"\nItem : "+item+"\nQuantity : "+quan+"\nShipped Date and Time : "+shipDate+"\nCity : "+city+"\nCost : "+cost+"\nEstimated Time of Delivery : "+est+"\nTHANK YOU!";
            MailApp.sendEmail(to, " Your Order is Shipped! ", message);
            MailApp.sendEmail(eto, " Your Order is Shipped! ", message);
        }
    }
  return ContentService.createTextOutput('success');
}