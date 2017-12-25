// Common Javascript functions

function openPopup(imageElement) {
 
    var div = document.getElementById("popup-div");
    var i = document.getElementById("enlarged-image");

    var w = window;
    var d = document;
    var e = d.documentElement;
    var g = d.getElementsByTagName('body')[0];
    var x = w.innerWidth || e.clientWidth || g.clientWidth;
    var y = w.innerHeight|| e.clientHeight|| g.clientHeight;      
    
    i.setAttribute("src", imageElement.getAttribute("src"));      
    
    if (x/y > imageElement.width / imageElement.height) {
      i.setAttribute("height", Math.floor(y * 0.95));        
    } else {
      i.setAttribute("width", Math.floor(x * 0.95));        
    }
    
    div.style.display = "block";
};

function closePopup() {
    var div = document.getElementById("popup-div");
    
    div.style.display = "none";
};