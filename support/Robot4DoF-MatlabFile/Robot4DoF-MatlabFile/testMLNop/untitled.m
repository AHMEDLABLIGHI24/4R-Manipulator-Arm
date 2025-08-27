function varargout = untitled(varargin)
%UNTITLED MATLAB code file for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to untitled_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      UNTITLED('CALLBACK') and UNTITLED('CALLBACK',hObject,...) call the
%      local function named CALLBACK in UNTITLED.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 24-Dec-2017 22:03:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
axes(handles.axes2);
imshow('D:\GiaiBaiTap\KyThuatRobot\FINAL\Final(NOP)\ZIP\testMLNop\HCMUTE.PNG');

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    ModelName = 'NOP';
    global var;
    t1=get(handles.slider1,'value'); %lay gia tri tu cac slider
    set(handles.edit1,'string',num2str(t1));%hien thi cac gia tri tu slider ra edit
    
    t2=get(handles.slider2,'value');
    set(handles.edit2,'string',num2str(t2));
    
    t3=get(handles.slider3,'value');
    set(handles.edit3,'string',num2str(t3));
    
    t4=get(handles.slider4,'value');
    set(handles.edit4,'string',num2str(t4));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t1)) %TRUYEN CAC THONG SO CUA CAC THANH SLIDER TRONG GUI VAO KHOI SLIDER GAIN
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t2))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t3))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t4))
    
    T1=[cosd(t1)    0      -sind(t1) 50*cosd(t1)    ; sind(t1) 0        cosd(t1)    50*sind(t1)     ; 0 -1 0 145   ; 0 0 0 1];
    T2=[cosd(t2) -sind(t2) 0         90*cosd(t2)    ; sind(t2) cosd(t2)  0          90*sind(t2)     ; 0  0 1 0     ; 0 0 0 1];
    T3=[cosd(t3) -sind(t3) 0         108*cosd(t3)   ; sind(t3) cosd(t3)  0          108*sind(t3)    ; 0  0 1 0     ; 0 0 0 1];
    T4=[cosd(t4) -sind(t4) 0         62.5*cosd(t4)  ; sind(t4) cosd(t4)  0          62.5*sind(t4)   ; 0  0 1 0     ; 0 0 0 1];
    T=T1*T2*T3*T4;
    Px=T(1,4);
    Py=T(2,4);
    Pz=T(3,4);
    
    set(handles.edit5,'string',num2str(Px));
    set(handles.edit6,'string',num2str(Py));
    set(handles.edit7,'string',num2str(Pz));

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    ModelName = 'NOP';
    global var;
    t1=get(handles.slider1,'value'); %lay gia tri tu cac slider
    set(handles.edit1,'string',num2str(t1));%hien thi cac gia tri tu slider ra edit
    
    t2=get(handles.slider2,'value');
    set(handles.edit2,'string',num2str(t2));
    
    t3=get(handles.slider3,'value');
    set(handles.edit3,'string',num2str(t3));
    
    t4=get(handles.slider4,'value');
    set(handles.edit4,'string',num2str(t4));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t1)) %TRUYEN CAC THONG SO CUA CAC THANH SLIDER TRONG GUI VAO KHOI SLIDER GAIN
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t2))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t3))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t4))
    
    T1=[cosd(t1)    0      -sind(t1) 50*cosd(t1)    ; sind(t1) 0        cosd(t1)    50*sind(t1)     ; 0 -1 0 145   ; 0 0 0 1];
    T2=[cosd(t2) -sind(t2) 0         90*cosd(t2)    ; sind(t2) cosd(t2)  0          90*sind(t2)     ; 0  0 1 0     ; 0 0 0 1];
    T3=[cosd(t3) -sind(t3) 0         108*cosd(t3)   ; sind(t3) cosd(t3)  0          108*sind(t3)    ; 0  0 1 0     ; 0 0 0 1];
    T4=[cosd(t4) -sind(t4) 0         62.5*cosd(t4)  ; sind(t4) cosd(t4)  0          62.5*sind(t4)   ; 0  0 1 0     ; 0 0 0 1];
    T=T1*T2*T3*T4;
    Px=T(1,4);
    Py=T(2,4);
    Pz=T(3,4);
    
    set(handles.edit5,'string',num2str(Px));
    set(handles.edit6,'string',num2str(Py));
    set(handles.edit7,'string',num2str(Pz));

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
     ModelName = 'NOP';
    global var;
    t1=get(handles.slider1,'value'); %lay gia tri tu cac slider
    set(handles.edit1,'string',num2str(t1));%hien thi cac gia tri tu slider ra edit
    
    t2=get(handles.slider2,'value');
    set(handles.edit2,'string',num2str(t2));
    
    t3=get(handles.slider3,'value');
    set(handles.edit3,'string',num2str(t3));
    
    t4=get(handles.slider4,'value');
    set(handles.edit4,'string',num2str(t4));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t1)) %TRUYEN CAC THONG SO CUA CAC THANH SLIDER TRONG GUI VAO KHOI SLIDER GAIN
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t2))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t3))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t4))
    
    T1=[cosd(t1)    0      -sind(t1) 50*cosd(t1)    ; sind(t1) 0        cosd(t1)    50*sind(t1)     ; 0 -1 0 145   ; 0 0 0 1];
    T2=[cosd(t2) -sind(t2) 0         90*cosd(t2)    ; sind(t2) cosd(t2)  0          90*sind(t2)     ; 0  0 1 0     ; 0 0 0 1];
    T3=[cosd(t3) -sind(t3) 0         108*cosd(t3)   ; sind(t3) cosd(t3)  0          108*sind(t3)    ; 0  0 1 0     ; 0 0 0 1];
    T4=[cosd(t4) -sind(t4) 0         62.5*cosd(t4)  ; sind(t4) cosd(t4)  0          62.5*sind(t4)   ; 0  0 1 0     ; 0 0 0 1];
    T=T1*T2*T3*T4;
    Px=T(1,4);
    Py=T(2,4);
    Pz=T(3,4);
    
    set(handles.edit5,'string',num2str(Px));
    set(handles.edit6,'string',num2str(Py));
    set(handles.edit7,'string',num2str(Pz));

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    ModelName = 'NOP';
    global var;
    t1=get(handles.slider1,'value'); %lay gia tri tu cac slider
    set(handles.edit1,'string',num2str(t1));%hien thi cac gia tri tu slider ra edit
    
    t2=get(handles.slider2,'value');
    set(handles.edit2,'string',num2str(t2));
    
    t3=get(handles.slider3,'value');
    set(handles.edit3,'string',num2str(t3));
    
    t4=get(handles.slider4,'value');
    set(handles.edit4,'string',num2str(t4));
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t1)) %TRUYEN CAC THONG SO CUA CAC THANH SLIDER TRONG GUI VAO KHOI SLIDER GAIN
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t2))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t3))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t4))
    
    T1=[cosd(t1)    0      -sind(t1) 50*cosd(t1)    ; sind(t1) 0        cosd(t1)    50*sind(t1)     ; 0 -1 0 145   ; 0 0 0 1];
    T2=[cosd(t2) -sind(t2) 0         90*cosd(t2)    ; sind(t2) cosd(t2)  0          90*sind(t2)     ; 0  0 1 0     ; 0 0 0 1];
    T3=[cosd(t3) -sind(t3) 0         108*cosd(t3)   ; sind(t3) cosd(t3)  0          108*sind(t3)    ; 0  0 1 0     ; 0 0 0 1];
    T4=[cosd(t4) -sind(t4) 0         62.5*cosd(t4)  ; sind(t4) cosd(t4)  0          62.5*sind(t4)   ; 0  0 1 0     ; 0 0 0 1];
    T=T1*T2*T3*T4;
    Px=T(1,4);
    Py=T(2,4);
    Pz=T(3,4);
    
    set(handles.edit5,'string',num2str(Px));
    set(handles.edit6,'string',num2str(Py));
    set(handles.edit7,'string',num2str(Pz));

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    Px_inv=get(handles.slider5,'value');
    set(handles.edit5,'string',num2str(Px_inv));

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
    Px_inv =str2double(get(handles.edit5,'string'));%lay cac gia tri cho Px_inv bang cach nhap so vao edit
    set(handles.slider5,'value',Px_inv);


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    Py_inv=get(handles.slider6,'value');
    set(handles.edit6,'string',num2str(Py_inv));

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double
    Py_inv =str2double(get(handles.edit6,'string'));%lay cac gia tri cho Px_inv bang cach nhap so vao edit
    set(handles.slider6,'value',Py_inv);


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    Pz_inv=get(handles.slider7,'value');
    set(handles.edit7,'string',num2str(Pz_inv));

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
    Pz_inv =str2double(get(handles.edit7,'string'));%lay cac gia tri cho Px_inv bang cach nhap so vao edit
    set(handles.slider7,'value',Pz_inv);


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in default.
function default_Callback(hObject, eventdata, handles)
% hObject    handle to default (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ModelName = 'NOP';
    global var;
    t1=0; 
    set(handles.edit1,'string',num2str(t1));%hien thi cac gia tri tu slider ra edit
    set(handles.slider1,'value',t1);%keo cac thanh slider tro ve vi tri 0
    t2=0;
    set(handles.edit2,'string',num2str(t2));
    set(handles.slider2,'value',t2);
    t3=0;
    set(handles.edit3,'string',num2str(t3));
    set(handles.slider3,'value',t3);
    t4=0;
    set(handles.edit4,'string',num2str(t4));
    set(handles.slider4,'value',t4);
    
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t1)) %TRUYEN CAC THONG SO CUA CAC THANH SLIDER TRONG GUI VAO KHOI SLIDER GAIN
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t2))
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t3))
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t4))
    
    T1=[cosd(t1)    0      -sind(t1) 50*cosd(t1)    ; sind(t1) 0        cosd(t1)    50*sind(t1)     ; 0 -1 0 145   ; 0 0 0 1];
    T2=[cosd(t2) -sind(t2) 0         90*cosd(t2)    ; sind(t2) cosd(t2)  0          90*sind(t2)     ; 0  0 1 0     ; 0 0 0 1];
    T3=[cosd(t3) -sind(t3) 0         108*cosd(t3)   ; sind(t3) cosd(t3)  0          108*sind(t3)    ; 0  0 1 0     ; 0 0 0 1];
    T4=[cosd(t4) -sind(t4) 0         62.5*cosd(t4)  ; sind(t4) cosd(t4)  0          62.5*sind(t4)   ; 0  0 1 0     ; 0 0 0 1];
    T=T1*T2*T3*T4;
    Px=T(1,4);
    Py=T(2,4);
    Pz=T(3,4);
    
    set(handles.edit5,'string',num2str(Px));
    set(handles.edit6,'string',num2str(Py));
    set(handles.edit7,'string',num2str(Pz));

% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    close;


% --- Executes on button press in Inverse.
function Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    ModelName = 'NOP';
    global var;
    Px_inv =str2double(get(handles.edit5,'string'));%lay cac gia tri cho Px_inv bang cach nhap so vao edit
    set(handles.slider5,'value',Px_inv);
    
    Py_inv =str2double(get(handles.edit6,'string'));%lay cac gia tri cho Py_inv bang cach nhap so vao edit
    set(handles.slider6,'value',Py_inv);
    
    Pz_inv =str2double(get(handles.edit7,'string'));%lay cac gia tri cho Pz_inv bang cach nhap so vao edit
    set(handles.slider7,'value',Pz_inv);
    
%     %% Lay vi tri cua robot tu cac thanh slider
%     Px_inv = get(handles.slider5,'value'); %lay gia tri tu cac slider
%     Inverse(handles.edit5,'string',num2str(Px_inv));%hien thi cac gia tri tu slider ra edit
%     
%     Py_inv = get(handles.slider6,'value');
%     Inverse(handles.edit6,'string',num2str(Py_inv));
%     
%     Pz_inv = get(handles.slider7,'value');
%     Inverse(handles.edit7,'string',num2str(Pz_inv));
    %%
    L1=50; L2=90; L3=108; L4=62.5;
    t1_inv= atan2d(Py_inv,Px_inv); %theta1
    
    nx=Px_inv*cosd(t1_inv)+Py_inv*sind(t1_inv)-L1-L4;
    ny=145-Pz_inv; %voi d1=145
    M=(nx*nx+ny*ny-L2*L2-L3*L3)/(2*L2*L3); %cos(theta3_inv)
    N=sqrt(1-M*M);%sin(theta3_inv) 
    t3_inv=atan2d(N,M);%theta3
    
    A=ny*(L3*cosd(t3_inv)+L2)-L3*sind(t3_inv)*nx;
    B=nx*(L3*cosd(t3_inv)+L2)+L3*sind(t3_inv)*ny;    
    t2_inv=atan2d(A,B);%theta2
    
    t4_inv= -t2_inv -t3_inv;%theta4
%% 
% t1_inv=atan2d(Py_inv,Px_inv);
% theta234= 0;
%  
% d=sqrt(Px_inv^2+Py_inv^2);
% xd=d*cosd(t1_inv);
% yd=d*sind(t1_inv);
% r4=d-L4*cosd(theta234);
% z4=Pz_inv-L4*sind(theta234);
% s=sqrt((z4-L1)^2+r4^2);
% t3_inv=acosd((s^2-L2^2-L3^2)/(2*L2*L3));
% beta=atan2d(L3*sind(t3_inv),L2+L3*cosd(t3_inv));
% alpha=atan2d(z4-L1,r4);
% t2_inv=alpha-beta;
% t4_inv=-t2_inv-t3_inv;     
%%
    guidata(hObject,handles);
    set(handles.edit1,'string',num2str(t1_inv));
    %Inverse(handles.slider1,'value',t1_inv);%keo thanh slider vao vi tri theta
    set_param([ModelName '/Slider Gain'],'Gain',num2str(t1_inv)) %TRUYEN CAC THONG SO CUA CAC THANH SLIDER TRONG GUI VAO KHOI SLIDER GAIN
    
    set(handles.edit2,'string',num2str(t2_inv));
    %Inverse(handles.slider2,'value',t2_inv);
    set_param([ModelName '/Slider Gain1'],'Gain',num2str(t2_inv))
    
    set(handles.edit3,'string',num2str(t3_inv));
    %Inverse(handles.slider3,'value',t3_inv);
    set_param([ModelName '/Slider Gain2'],'Gain',num2str(t3_inv))
    
    set(handles.edit4,'string',num2str(t4_inv));
    %Inverse(handles.slider4,'value',t4_inv);
    set_param([ModelName '/Slider Gain3'],'Gain',num2str(t4_inv))
