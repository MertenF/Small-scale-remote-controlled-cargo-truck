# Small-scale-remote-controlled-cargo-truck
Maarten Herremans & Merten Fermont

## Omschrijving
Een op afstand bestuurbare cargo truck waarmee een bak bier vervoerd kan worden.
De afstandsbediening en de truck zelf zijn de 2 grote blokken van het project. Beiden zijn gecentreerd rond de STM32F303R68T microcontroller.

## Truck
De truck beweegt zich door middel van 2 BLDC motoren en stuurt met een servo motor.
Er zijn voor en achterlichten aanwezig voor in het donker, samen met pinkers. Om in het donker goed te zien waar de truck rijd is er powerled als fare voorzien. Een toeter is ook voorzien.
Dit zal allemaal gevoed worden door een 5s13p batterijpack van lithiumcellen, beveiligd met een BMS.

##Afstandsbediening
Het afstandsbediening bevat 2 joysticks om gas te geven of te remmen en te sturen. Een aantal drukknoppen om de lichten te bedienen en een rotary encoder om de helderheid van de fare in te stellen.
De communicatie gebeurd op de 2.4GHz band met de NRF24L01+ chip